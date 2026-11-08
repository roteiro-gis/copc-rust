//! Disk-backed LOD octree index construction for COPC writes.

use std::collections::hash_map::{Entry, HashMap};
use std::fs::File;
use std::hash::{BuildHasherDefault, Hasher};
use std::io::{BufReader, BufWriter, Write};
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
/// Interior nodes select representatives on grids of up to `2^8 = 256` cells
/// per axis; see [`GridSampler`].
const LOD_GRID_MAX_LEVEL: u32 = 8;

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

/// A temporary file of ascending source indexes, minus the sorted `exclude`
/// indexes that an ancestor node already took as representatives.
struct IndexRun {
    path: TempPath,
    records: usize,
    exclude: Vec<u32>,
}

impl IndexRun {
    fn len(&self) -> usize {
        self.records - self.exclude.len()
    }

    /// Visit every index in the run that is not excluded, in ascending order.
    fn for_each(
        &self,
        cancel: &dyn CancelCheck,
        mut visit: impl FnMut(u32) -> Result<()>,
    ) -> Result<()> {
        let path: &Path = self.path.as_ref();
        let file = File::open(path).map_err(|e| Error::io("open LOD index", e))?;
        let mut reader = BufReader::with_capacity(INDEX_IO_BUFFER_BYTES, file);
        let mut excluded = self.exclude.iter().copied().peekable();
        for read_index in 0..self.records {
            if read_index.is_multiple_of(CANCEL_POLL_STRIDE) {
                cancel.check()?;
            }
            let index = reader
                .read_u32::<LittleEndian>()
                .map_err(|e| Error::io("read LOD index", e))?;
            if excluded.next_if_eq(&index).is_some() {
                continue;
            }
            visit(index)?;
        }
        if excluded.next().is_some() {
            return Err(Error::InvalidData(
                "LOD index exclusions are not a subset of their run".into(),
            ));
        }
        Ok(())
    }
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

/// COPC spacing for an octree whose root cube has `halfsize`: the cell size
/// of the finest grid used to select root representatives.
pub(crate) fn lod_spacing(halfsize: f64) -> f64 {
    2.0 * halfsize / f64::from(1u32 << LOD_GRID_MAX_LEVEL)
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
        let count = run.len();
        if count == 0 {
            return Ok(());
        }
        let start = *self.order_offset;
        if count <= self.max_points_per_node {
            let (order_writer, order_offset) = (&mut *self.order_writer, &mut *self.order_offset);
            run.for_each(self.cancel, |index| {
                append_index_to_order(order_writer, order_offset, index)
            })?;
            self.nodes.push(LodNodeRange { key, start, count });
            return Ok(());
        }
        if key.level as u32 >= MAX_OCTREE_DEPTH {
            return Err(Error::InvalidInput(format!(
                "octree node {key:?} still contains {count} points at the maximum depth; increase max_points_per_node above {}",
                self.max_points_per_node
            )));
        }

        let Partition { children, selected } = partition_index_run(
            self.source,
            &run,
            bounds,
            self.max_points_per_node,
            self.cancel,
        )?;
        drop(run);
        for &(index, _) in &selected {
            append_index_to_order(self.order_writer, self.order_offset, index)?;
        }
        self.nodes.push(LodNodeRange {
            key,
            start,
            count: selected.len(),
        });

        // `selected` is sorted by index, so each per-octant list is too.
        let mut excludes: [_; 8] = std::array::from_fn(|_| Vec::new());
        for (index, octant) in selected {
            excludes[usize::from(octant)].push(index);
        }
        for (octant, (child, exclude)) in children.into_iter().zip(excludes).enumerate() {
            let Some(mut child) = child else {
                continue;
            };
            child.exclude = exclude;
            if child.len() == 0 {
                continue;
            }
            self.assign(key.child(octant as u8)?, child, bounds.octant(octant as u8))?;
        }
        Ok(())
    }
}

fn write_root_index_run(total_points: u32, cancel: &dyn CancelCheck) -> Result<IndexRun> {
    let mut writer = BufWriter::with_capacity(INDEX_IO_BUFFER_BYTES, new_index_tempfile("root")?);
    for index in 0..total_points {
        if (index as usize).is_multiple_of(CANCEL_POLL_STRIDE) {
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
        records: total_points as usize,
        exclude: Vec::new(),
    })
}

struct Partition {
    children: [Option<IndexRun>; 8],
    /// Representatives kept by the parent node as `(index, octant)`, sorted
    /// by index.
    selected: Vec<(u32, u8)>,
}

/// Split a run into its eight child octants while choosing the parent's
/// representative points in the same pass.
fn partition_index_run<S: CopcPointSource>(
    source: &S,
    run: &IndexRun,
    bounds: Bounds,
    max_points_per_node: usize,
    cancel: &dyn CancelCheck,
) -> Result<Partition> {
    let mut writers: [Option<BufWriter<NamedTempFile>>; 8] = std::array::from_fn(|_| None);
    let mut counts = [0usize; 8];
    let center = bounds.center();
    let mut sampler = GridSampler::new(bounds, max_points_per_node);
    run.for_each(cancel, |index| {
        let (x, y, z) = source.xyz(index as usize)?;
        let octant = child_octant(center, x, y, z);
        let writer = match &mut writers[octant] {
            Some(writer) => writer,
            empty => empty.insert(BufWriter::with_capacity(
                INDEX_IO_BUFFER_BYTES,
                new_index_tempfile("partition")?,
            )),
        };
        writer
            .write_u32::<LittleEndian>(index)
            .map_err(|e| Error::io("write LOD partition index", e))?;
        counts[octant] += 1;
        sampler.offer(index, octant as u8, (x, y, z));
        Ok(())
    })?;

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
            records: counts[octant],
            exclude: Vec::new(),
        });
    }
    Ok(Partition {
        children,
        selected: sampler.into_selection(),
    })
}

/// Chooses an interior node's representative points: the first point seen in
/// each occupied cell of the finest power-of-two voxel grid over the node
/// whose occupied-cell count fits the node's point budget. Selecting by
/// occupied cell rather than by input order spreads every LOD level evenly
/// over the node, regardless of how the source points are ordered.
struct GridSampler {
    min: (f64, f64, f64),
    cells_per_unit: (f64, f64, f64),
    budget: usize,
    /// `levels[k]` maps each occupied cell of the `2^k`-per-axis grid to its
    /// representative `(index, octant)`. Levels whose occupancy exceeds the
    /// budget are dropped, so the last level is always the finest that fits.
    levels: Vec<HashMap<u32, (u32, u8), BuildHasherDefault<CellHasher>>>,
}

impl GridSampler {
    fn new(bounds: Bounds, budget: usize) -> Self {
        let cells = f64::from(1u32 << LOD_GRID_MAX_LEVEL);
        Self {
            min: bounds.min,
            cells_per_unit: (
                cells / (bounds.max.0 - bounds.min.0),
                cells / (bounds.max.1 - bounds.min.1),
                cells / (bounds.max.2 - bounds.min.2),
            ),
            budget: budget.max(1),
            levels: (0..=LOD_GRID_MAX_LEVEL)
                .map(|_| HashMap::default())
                .collect(),
        }
    }

    fn offer(&mut self, index: u32, octant: u8, (x, y, z): (f64, f64, f64)) {
        let max_cell = (1u32 << LOD_GRID_MAX_LEVEL) - 1;
        // Float-to-int casts saturate (NaN maps to 0), so edge points clamp
        // into the outermost cells.
        let cell =
            |value: f64, min: f64, per_unit: f64| (((value - min) * per_unit) as u32).min(max_cell);
        let cx = cell(x, self.min.0, self.cells_per_unit.0);
        let cy = cell(y, self.min.1, self.cells_per_unit.1);
        let cz = cell(z, self.min.2, self.cells_per_unit.2);
        // An occupied cell implies its coarser ancestors are occupied, so
        // stop at the first level that already has a representative.
        for (level, cells) in self.levels.iter_mut().enumerate().rev() {
            let shift = LOD_GRID_MAX_LEVEL - level as u32;
            let key = ((cx >> shift) << (2 * LOD_GRID_MAX_LEVEL))
                | ((cy >> shift) << LOD_GRID_MAX_LEVEL)
                | (cz >> shift);
            match cells.entry(key) {
                Entry::Occupied(_) => break,
                Entry::Vacant(cell) => {
                    cell.insert((index, octant));
                }
            }
        }
        while self.levels.len() > 1 && self.levels.last().map_or(0, HashMap::len) > self.budget {
            self.levels.pop();
        }
    }

    fn into_selection(mut self) -> Vec<(u32, u8)> {
        let mut selected: Vec<_> = self
            .levels
            .pop()
            .unwrap_or_default()
            .into_values()
            .collect();
        selected.sort_unstable_by_key(|&(index, _)| index);
        selected
    }
}

/// Hasher for packed grid-cell keys: a splitmix64 finalizer, which is much
/// cheaper than SipHash and spreads the structured keys across buckets.
#[derive(Default)]
struct CellHasher(u64);

impl Hasher for CellHasher {
    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 = self.0.rotate_left(8) ^ u64::from(*byte);
        }
    }

    fn write_u32(&mut self, value: u32) {
        self.0 = u64::from(value);
    }

    fn finish(&self) -> u64 {
        let mut z = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
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

    use std::io::{Seek, SeekFrom};

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
    fn interior_nodes_cover_the_whole_node_for_scan_ordered_input() {
        // Row-major input is the worst case for input-order selection: the
        // first points of each octant are a thin strip along its bottom edge.
        let side = 100u32;
        let points = (0..side * side)
            .map(|i| CopcPointFields {
                x: f64::from(i % side),
                y: f64::from(i / side),
                return_number: 1,
                number_of_returns: 1,
                ..CopcPointFields::default()
            })
            .collect();
        let source = VecSource { points };
        let bounds = source_bounds(&source);
        let (center, halfsize) = cube_from_bounds(&bounds);
        let params = CopcWriterParams::new(500);

        let lod = build_lod_index(&source, center, halfsize, &params, &NeverCancel).unwrap();
        let ranges = read_lod_index(&lod).unwrap();
        let (key, root) = &ranges[0];
        assert_eq!(VoxelKey::root(), *key);
        assert!(root.len() <= 500);

        let mut coarse_cells = std::collections::HashSet::new();
        for &index in root {
            let point = &source.points[index as usize];
            coarse_cells.insert(((point.x / 10.0) as u32, (point.y / 10.0) as u32));
        }
        assert_eq!(
            100,
            coarse_cells.len(),
            "root representatives must touch every 10x10 cell of the cloud"
        );
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
