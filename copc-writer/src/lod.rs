//! Disk-backed LOD octree index construction for COPC writes.

use std::fs::File;
use std::io::{BufReader, BufWriter, Seek, SeekFrom, Write};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use copc_core::{Bounds, CancelCheck, Error, Result, VoxelKey};
use tempfile::{NamedTempFile, TempPath};

use crate::source::CopcPointSource;
use crate::writer::CopcWriterParams;
use crate::CANCEL_POLL_STRIDE;

pub(crate) const INDEX_RECORD_BYTES: u64 = 4;
pub(crate) const INDEX_IO_BUFFER_BYTES: usize = 1024 * 1024;
/// Hard cap on octree subdivision depth: deeper voxel keys would overflow the
/// i32 key coordinates (level 30 keys reach 2^30). The layered LAZ compressor
/// buffers an entire COPC chunk (one octree node) in memory before flushing,
/// so nodes must keep subdividing until they fit `max_points_per_node`.
/// Pathological coincident inputs that cannot fit by this depth are rejected
/// rather than producing an oversized chunk.
const MAX_OCTREE_DEPTH: u32 = 30;

pub(crate) struct LodIndex {
    pub(crate) nodes: Vec<LodNodeRange>,
    pub(crate) order_path: TempPath,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LodNodeRange {
    pub(crate) key: VoxelKey,
    pub(crate) start: u64,
    pub(crate) count: usize,
}

struct IndexRun {
    path: TempPath,
    start: u64,
    count: usize,
}

pub(crate) fn build_lod_index<S: CopcPointSource>(
    source: &S,
    center: (f64, f64, f64),
    halfsize: f64,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
) -> Result<LodIndex> {
    cancel.check()?;
    let total_points = u32::try_from(source.len()).map_err(|_| {
        Error::InvalidInput("COPC writer supports at most u32::MAX points per file".into())
    })?;
    if params.max_points_per_node == 0 {
        return Err(Error::InvalidInput(
            "max_points_per_node must be greater than zero".into(),
        ));
    }
    let max_points_per_node = params.max_points_per_node as usize;
    let root_run = write_root_index_run(total_points, cancel)?;
    let mut order_file = new_index_tempfile("order")?;
    let mut order_offset = 0;
    let mut nodes = Vec::new();
    {
        let mut order_writer =
            BufWriter::with_capacity(INDEX_IO_BUFFER_BYTES, order_file.as_file_mut());
        let mut builder = LodIndexBuilder {
            source,
            max_points_per_node,
            cancel,
            order_writer: &mut order_writer,
            order_offset: &mut order_offset,
            nodes: &mut nodes,
        };
        builder.assign(VoxelKey::root(), root_run, Bounds::cube(center, halfsize))?;
        order_writer
            .flush()
            .map_err(|e| Error::io("flush LOD index order", e))?;
    }
    nodes.sort_by_key(|node| node.key);
    Ok(LodIndex {
        nodes,
        order_path: order_file.into_temp_path(),
    })
}

struct LodIndexBuilder<'a, S: CopcPointSource, W: Write> {
    source: &'a S,
    max_points_per_node: usize,
    cancel: &'a dyn CancelCheck,
    order_writer: &'a mut W,
    order_offset: &'a mut u64,
    nodes: &'a mut Vec<LodNodeRange>,
}

impl<S: CopcPointSource, W: Write> LodIndexBuilder<'_, S, W> {
    fn assign(&mut self, key: VoxelKey, run: IndexRun, bounds: Bounds) -> Result<()> {
        self.cancel.check()?;
        if run.count == 0 {
            return Ok(());
        }
        if run.count <= self.max_points_per_node {
            let start = *self.order_offset;
            append_index_run_to_order(&run, self.order_writer, self.order_offset, self.cancel)?;
            self.nodes.push(LodNodeRange {
                key,
                start,
                count: run.count,
            });
            return Ok(());
        }
        if key.level as u32 >= MAX_OCTREE_DEPTH {
            return Err(Error::InvalidInput(format!(
                "octree node {key:?} still contains {} points at the maximum depth; increase max_points_per_node above {}",
                run.count, self.max_points_per_node
            )));
        }

        let mut children = partition_index_run(self.source, &run, bounds, self.cancel)?;
        let start = *self.order_offset;
        let selected_counts = append_lod_selection_to_order(
            &children,
            self.max_points_per_node,
            self.order_writer,
            self.order_offset,
            self.cancel,
        )?;
        let selected_total = selected_counts.iter().sum();
        self.nodes.push(LodNodeRange {
            key,
            start,
            count: selected_total,
        });

        for (octant, child) in children.iter_mut().enumerate() {
            let Some(mut child_run) = child.take() else {
                continue;
            };
            let selected = selected_counts[octant];
            if selected >= child_run.count {
                continue;
            }
            child_run.start += selected as u64 * INDEX_RECORD_BYTES;
            child_run.count -= selected;
            self.assign(
                key.child(octant as u8)?,
                child_run,
                bounds.octant(octant as u8),
            )?;
        }
        Ok(())
    }
}

fn write_root_index_run(total_points: u32, cancel: &dyn CancelCheck) -> Result<IndexRun> {
    let mut writer = BufWriter::with_capacity(INDEX_IO_BUFFER_BYTES, new_index_tempfile("root")?);
    for index in 0..total_points {
        if index as usize % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        writer
            .write_u32::<LittleEndian>(index)
            .map_err(|e| Error::io("write root LOD index", e))?;
    }
    let file = writer
        .into_inner()
        .map_err(|e| Error::io("flush root LOD index", e.into_error()))?;
    Ok(IndexRun {
        path: file.into_temp_path(),
        start: 0,
        count: total_points as usize,
    })
}

fn partition_index_run<S: CopcPointSource>(
    source: &S,
    run: &IndexRun,
    bounds: Bounds,
    cancel: &dyn CancelCheck,
) -> Result<[Option<IndexRun>; 8]> {
    let mut reader = open_index_run(run)?;
    let mut writers: [Option<BufWriter<NamedTempFile>>; 8] = std::array::from_fn(|_| None);
    let mut counts = [0usize; 8];
    let center = bounds.center();
    for read_index in 0..run.count {
        if read_index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let index = reader
            .read_u32::<LittleEndian>()
            .map_err(|e| Error::io("read LOD partition index", e))?;
        let (x, y, z) = source.xyz(index as usize)?;
        let octant = child_octant(center, x, y, z);
        if writers[octant].is_none() {
            writers[octant] = Some(BufWriter::with_capacity(
                INDEX_IO_BUFFER_BYTES,
                new_index_tempfile("partition")?,
            ));
        }
        writers[octant]
            .as_mut()
            .ok_or_else(|| Error::InvalidData("partition writer was not created".into()))?
            .write_u32::<LittleEndian>(index)
            .map_err(|e| Error::io("write LOD partition index", e))?;
        counts[octant] += 1;
    }

    let mut children: [Option<IndexRun>; 8] = std::array::from_fn(|_| None);
    for octant in 0..8 {
        let Some(writer) = writers[octant].take() else {
            continue;
        };
        let file = writer
            .into_inner()
            .map_err(|e| Error::io("flush LOD partition index", e.into_error()))?;
        children[octant] = Some(IndexRun {
            path: file.into_temp_path(),
            start: 0,
            count: counts[octant],
        });
    }
    Ok(children)
}

fn append_lod_selection_to_order<W: Write>(
    children: &[Option<IndexRun>; 8],
    max_points_per_node: usize,
    order_writer: &mut W,
    order_offset: &mut u64,
    cancel: &dyn CancelCheck,
) -> Result<[usize; 8]> {
    let mut readers: [Option<BufReader<File>>; 8] = std::array::from_fn(|_| None);
    for octant in 0..8 {
        if let Some(child) = &children[octant] {
            readers[octant] = Some(open_index_run(child)?);
        }
    }

    let mut selected_counts = [0usize; 8];
    let mut selected_total = 0usize;
    while selected_total < max_points_per_node {
        cancel.check()?;
        let mut progressed = false;
        for octant in 0..8 {
            let Some(child) = &children[octant] else {
                continue;
            };
            if selected_counts[octant] >= child.count {
                continue;
            }
            let index = readers[octant]
                .as_mut()
                .ok_or_else(|| Error::InvalidData("partition reader was not opened".into()))?
                .read_u32::<LittleEndian>()
                .map_err(|e| Error::io("read selected LOD index", e))?;
            append_index_to_order(order_writer, order_offset, index)?;
            selected_counts[octant] += 1;
            selected_total += 1;
            progressed = true;
            if selected_total == max_points_per_node {
                break;
            }
        }
        if !progressed {
            break;
        }
    }
    Ok(selected_counts)
}

fn append_index_run_to_order<W: Write>(
    run: &IndexRun,
    order_writer: &mut W,
    order_offset: &mut u64,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    let mut reader = open_index_run(run)?;
    for read_index in 0..run.count {
        if read_index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let index = reader
            .read_u32::<LittleEndian>()
            .map_err(|e| Error::io("read LOD index", e))?;
        append_index_to_order(order_writer, order_offset, index)?;
    }
    Ok(())
}

fn append_index_to_order<W: Write>(
    order_writer: &mut W,
    order_offset: &mut u64,
    index: u32,
) -> Result<()> {
    order_writer
        .write_u32::<LittleEndian>(index)
        .map_err(|e| Error::io("write LOD index order", e))?;
    *order_offset = order_offset
        .checked_add(INDEX_RECORD_BYTES)
        .ok_or_else(|| Error::InvalidInput("LOD index order exceeds u64 range".into()))?;
    Ok(())
}

fn open_index_run(run: &IndexRun) -> Result<BufReader<File>> {
    let path: &Path = run.path.as_ref();
    let mut file = File::open(path).map_err(|e| Error::io("open LOD index", e))?;
    file.seek(SeekFrom::Start(run.start))
        .map_err(|e| Error::io("seek LOD index", e))?;
    Ok(BufReader::with_capacity(INDEX_IO_BUFFER_BYTES, file))
}

fn new_index_tempfile(label: &str) -> Result<NamedTempFile> {
    let prefix = format!(".copc-writer-{label}.");
    tempfile::Builder::new()
        .prefix(&prefix)
        .suffix(".idx")
        .tempfile()
        .map_err(|e| Error::io("create LOD index file", e))
}

fn child_octant(center: (f64, f64, f64), x: f64, y: f64, z: f64) -> usize {
    usize::from(x >= center.0)
        | (usize::from(y >= center.1) << 1)
        | (usize::from(z >= center.2) << 2)
}

pub(crate) fn cube_from_bounds(bounds: &Bounds) -> ((f64, f64, f64), f64) {
    let dx = bounds.max.0 - bounds.min.0;
    let dy = bounds.max.1 - bounds.min.1;
    let dz = bounds.max.2 - bounds.min.2;
    let center = (
        bounds.min.0 + dx * 0.5,
        bounds.min.1 + dy * 0.5,
        bounds.min.2 + dz * 0.5,
    );
    let halfsize = (dx.max(dy).max(dz) * 0.5).max(1e-6);
    (center, halfsize)
}

#[cfg(test)]
mod tests {
    use super::*;

    use copc_core::NeverCancel;

    use crate::source::CopcPointFields;

    struct VecSource {
        points: Vec<CopcPointFields>,
    }

    impl CopcPointSource for VecSource {
        fn len(&self) -> usize {
            self.points.len()
        }

        fn xyz(&self, index: usize) -> Result<(f64, f64, f64)> {
            let point = &self.points[index];
            Ok((point.x, point.y, point.z))
        }

        fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> Result<()> {
            out.clone_from(&self.points[index]);
            Ok(())
        }
    }

    #[test]
    fn spooled_lod_index_covers_each_point_once() {
        let points = (0..257)
            .map(|i| CopcPointFields {
                x: f64::from((i * 37) % 101),
                y: f64::from((i * 53) % 103),
                z: f64::from((i * 71) % 107),
                intensity: 0,
                return_number: 1,
                number_of_returns: 1,
                synthetic: 0,
                key_point: 0,
                withheld: 0,
                overlap: 0,
                scan_channel: 0,
                scan_direction_flag: 0,
                edge_of_flight_line: 0,
                classification: 0,
                user_data: 0,
                scan_angle: 0.0,
                point_source_id: 0,
                gps_time: f64::from(i),
                red: 0,
                green: 0,
                blue: 0,
                extra_bytes: Vec::new(),
            })
            .collect();
        let source = VecSource { points };
        let bounds = source_bounds(&source);
        let (center, halfsize) = cube_from_bounds(&bounds);
        let params = CopcWriterParams::new(7);

        let spooled = build_lod_index(&source, center, halfsize, &params, &NeverCancel).unwrap();
        let ranges = read_lod_index(&spooled).unwrap();

        let mut seen = vec![false; source.len()];
        let mut total = 0usize;
        for (_key, indices) in ranges {
            assert!(indices.len() <= params.max_points_per_node as usize);
            for index in indices {
                let seen = &mut seen[index as usize];
                assert!(!*seen, "point index {index} was assigned more than once");
                *seen = true;
                total += 1;
            }
        }
        assert_eq!(source.len(), total);
        assert!(seen.into_iter().all(|value| value));
    }

    #[test]
    fn dense_cluster_stays_bounded_below_giant_chunks() {
        // A dense cluster inside large bounds must keep subdividing until every
        // node fits `max_points_per_node`; an oversized leaf would force the
        // layered LAZ compressor to buffer that entire chunk in memory (the
        // multi-GB failure mode on real clouds).
        let field = |x: f64, y: f64, z: f64, i: u32| CopcPointFields {
            x,
            y,
            z,
            intensity: 0,
            return_number: 1,
            number_of_returns: 1,
            synthetic: 0,
            key_point: 0,
            withheld: 0,
            overlap: 0,
            scan_channel: 0,
            scan_direction_flag: 0,
            edge_of_flight_line: 0,
            classification: 0,
            user_data: 0,
            scan_angle: 0.0,
            point_source_id: 0,
            gps_time: f64::from(i),
            red: 0,
            green: 0,
            blue: 0,
            extra_bytes: Vec::new(),
        };
        // 4000 distinct points packed into a ~0.4-unit cluster ...
        let mut points: Vec<CopcPointFields> = (0..4_000u32)
            .map(|i| {
                let f = f64::from(i);
                field(
                    f * 1e-4,
                    (f * 1.7).fract() * 0.4,
                    (f * 2.3).fract() * 0.4,
                    i,
                )
            })
            .collect();
        // ... plus a few points spread wide to set large bounds around it.
        for i in 0..8u32 {
            points.push(field(
                f64::from(i) * 1000.0,
                f64::from(i) * 1000.0,
                f64::from(i) * 100.0,
                100_000 + i,
            ));
        }
        let max_points = 100usize;
        let source = VecSource { points };
        let bounds = source_bounds(&source);
        let (center, halfsize) = cube_from_bounds(&bounds);
        let params = CopcWriterParams::new(max_points as u32);

        let lod = build_lod_index(&source, center, halfsize, &params, &NeverCancel).unwrap();
        for (key, indices) in read_lod_index(&lod).unwrap() {
            assert!(
                indices.len() <= max_points,
                "node {key:?} holds {} points, exceeding max_points_per_node {max_points}",
                indices.len(),
            );
        }
    }

    #[test]
    fn identical_points_fail_instead_of_creating_an_unbounded_leaf() {
        let point = CopcPointFields {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            return_number: 1,
            number_of_returns: 1,
            ..CopcPointFields::default()
        };
        let source = VecSource {
            points: vec![point; 32],
        };
        let bounds = source_bounds(&source);
        let (center, halfsize) = cube_from_bounds(&bounds);
        let error = build_lod_index(
            &source,
            center,
            halfsize,
            &CopcWriterParams::new(1),
            &NeverCancel,
        )
        .err()
        .expect("identical points must exceed the depth cap");

        assert!(error.to_string().contains("maximum depth"));
    }

    fn source_bounds(source: &VecSource) -> Bounds {
        source.points.iter().fold(
            Bounds::point(source.points[0].x, source.points[0].y, source.points[0].z),
            |mut bounds, point| {
                bounds.extend(point.x, point.y, point.z);
                bounds
            },
        )
    }

    fn read_lod_index(index: &LodIndex) -> Result<Vec<(VoxelKey, Vec<u32>)>> {
        let path: &Path = index.order_path.as_ref();
        let mut reader =
            BufReader::new(File::open(path).map_err(|e| Error::io("open LOD order", e))?);
        let mut out = Vec::new();
        for node in &index.nodes {
            reader
                .seek(SeekFrom::Start(node.start))
                .map_err(|e| Error::io("seek LOD order", e))?;
            let mut indices = Vec::with_capacity(node.count);
            for _ in 0..node.count {
                indices.push(
                    reader
                        .read_u32::<LittleEndian>()
                        .map_err(|e| Error::io("read LOD order", e))?,
                );
            }
            out.push((node.key, indices));
        }
        Ok(out)
    }
}
