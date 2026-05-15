use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;

use copc_core::{Bounds, CancelCheck, CopcInfo, Entry, Error, Result};
use las::point::Format as LasPointFormat;
use las::{Point, Transform, Vector};
use laz::record::{LayeredPointRecordDecompressor, RecordDecompressor};
use laz::LazVlr;

use crate::{CopcFile, LasHeader};

const CANCEL_POLL_STRIDE: usize = 4_096;

/// COPC point reader owning the underlying stream.
pub struct CopcReader<R> {
    source: R,
    file: CopcFile,
}

/// Limits the octree levels included in a point query.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LodSelection {
    /// Full resolution: every point chunk in every LOD.
    All,
    /// Include levels needed to satisfy the requested spacing.
    Resolution(f64),
    /// Include exactly one octree level.
    Level(i32),
    /// Include levels in `[min, max)`.
    LevelMinMax(i32, i32),
}

/// Limits points by XYZ bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoundsSelection {
    /// No bounds filter.
    All,
    /// Include points within the supplied bounds.
    Within(Bounds),
}

/// Point query used by [`CopcReader::points_for_query`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointQuery {
    pub lod: LodSelection,
    pub bounds: BoundsSelection,
}

impl PointQuery {
    pub const fn all() -> Self {
        Self {
            lod: LodSelection::All,
            bounds: BoundsSelection::All,
        }
    }

    pub const fn new(lod: LodSelection, bounds: BoundsSelection) -> Self {
        Self { lod, bounds }
    }
}

impl CopcReader<BufReader<File>> {
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path.as_ref()).map_err(|e| Error::io("open COPC file", e))?;
        Self::open(BufReader::new(file))
    }
}

impl<R: Read + Seek + Send> CopcReader<R> {
    /// Open a COPC reader from an already-open stream.
    pub fn open(mut source: R) -> Result<Self> {
        let file = CopcFile::from_reader(&mut source)?;
        Ok(Self { source, file })
    }

    pub fn file(&self) -> &CopcFile {
        &self.file
    }

    pub fn header(&self) -> &LasHeader {
        self.file.header()
    }

    pub fn copc_info(&self) -> &CopcInfo {
        self.file.copc_info()
    }

    pub fn into_inner(self) -> R {
        self.source
    }

    /// Iterate points matching the requested LOD and bounds.
    ///
    /// The iterator yields `Result<las::Point>` so IO, LAZ, and malformed-point
    /// failures are reported to the caller instead of panicking mid-stream.
    pub fn points(
        &mut self,
        lod: LodSelection,
        bounds: BoundsSelection,
    ) -> Result<PointIter<'_, R>> {
        self.points_for_query(PointQuery::new(lod, bounds))
    }

    pub fn points_for_query(&mut self, query: PointQuery) -> Result<PointIter<'_, R>> {
        PointIter::new(&mut self.source, &self.file, query, None)
    }

    pub fn points_with_cancel<'a>(
        &'a mut self,
        lod: LodSelection,
        bounds: BoundsSelection,
        cancel: &'a dyn CancelCheck,
    ) -> Result<PointIter<'a, R>> {
        PointIter::new(
            &mut self.source,
            &self.file,
            PointQuery::new(lod, bounds),
            Some(cancel),
        )
    }
}

/// Iterator over selected COPC point chunks.
pub struct PointIter<'a, R: Read + Seek + Send> {
    chunks: Vec<Entry>,
    next_chunk: usize,
    current_chunk_points_left: usize,
    remaining_candidate_points: usize,
    exact_size: bool,
    point_format: LasPointFormat,
    transforms: Vector<Transform>,
    bounds: Option<Bounds>,
    decoder: ChunkLazDecoder<'a, R>,
    point_buf: Vec<u8>,
    decoded_points: usize,
    cancel: Option<&'a dyn CancelCheck>,
    finished: bool,
}

impl<'a, R: Read + Seek + Send> PointIter<'a, R> {
    fn new(
        source: &'a mut R,
        file: &CopcFile,
        query: PointQuery,
        cancel: Option<&'a dyn CancelCheck>,
    ) -> Result<Self> {
        let chunks = select_point_chunks(file, query)?;
        let point_format = file.point_format()?;
        let transforms = file.transforms();
        let bounds = match query.bounds {
            BoundsSelection::All => None,
            BoundsSelection::Within(bounds) => Some(bounds),
        };
        let decoder = ChunkLazDecoder::new(source, file.laszip_vlr().clone())?;
        let expected_record_size = usize::from(point_format.len());
        if decoder.record_size() != expected_record_size {
            return Err(Error::InvalidData(format!(
                "LASzip item size is {} bytes, but LAS point record length is {} bytes",
                decoder.record_size(),
                expected_record_size
            )));
        }
        let remaining_candidate_points = total_candidate_points(&chunks)?;
        let point_buf = vec![0u8; expected_record_size];
        Ok(Self {
            chunks,
            next_chunk: 0,
            current_chunk_points_left: 0,
            remaining_candidate_points,
            exact_size: bounds.is_none(),
            point_format,
            transforms,
            bounds,
            decoder,
            point_buf,
            decoded_points: 0,
            cancel,
            finished: false,
        })
    }

    fn load_next_chunk(&mut self) -> Result<bool> {
        while self.next_chunk < self.chunks.len() {
            let entry = self.chunks[self.next_chunk];
            self.next_chunk += 1;
            if entry.point_count <= 0 {
                continue;
            }
            self.decoder.seek_to_chunk(entry.offset)?;
            self.current_chunk_points_left = usize::try_from(entry.point_count).map_err(|_| {
                Error::InvalidData(format!(
                    "negative point count {} for {:?}",
                    entry.point_count, entry.key
                ))
            })?;
            return Ok(true);
        }
        Ok(false)
    }

    fn next_inner(&mut self) -> Result<Option<Point>> {
        loop {
            while self.current_chunk_points_left == 0 {
                if !self.load_next_chunk()? {
                    return Ok(None);
                }
            }

            if self.decoded_points % CANCEL_POLL_STRIDE == 0 {
                if let Some(cancel) = self.cancel {
                    cancel.check()?;
                }
            }

            self.decoder.decompress_one(&mut self.point_buf)?;
            self.current_chunk_points_left -= 1;
            self.remaining_candidate_points -= 1;
            self.decoded_points += 1;

            let raw_point =
                las::raw::Point::read_from(self.point_buf.as_slice(), &self.point_format)
                    .map_err(|e| Error::Las(e.to_string()))?;
            if let Some(bounds) = self.bounds {
                let x = self.transforms.x.direct(raw_point.x);
                let y = self.transforms.y.direct(raw_point.y);
                let z = self.transforms.z.direct(raw_point.z);
                if !bounds.contains_xyz(x, y, z) {
                    continue;
                }
            }
            return Ok(Some(Point::new(raw_point, &self.transforms)));
        }
    }
}

impl<R: Read + Seek + Send> Iterator for PointIter<'_, R> {
    type Item = Result<Point>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        match self.next_inner() {
            Ok(Some(point)) => Some(Ok(point)),
            Ok(None) => {
                self.finished = true;
                None
            }
            Err(error) => {
                self.finished = true;
                Some(Err(error))
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        if self.exact_size {
            (
                self.remaining_candidate_points,
                Some(self.remaining_candidate_points),
            )
        } else {
            (0, Some(self.remaining_candidate_points))
        }
    }
}

struct ChunkLazDecoder<'a, R: Read + Seek + Send> {
    laz_vlr: LazVlr,
    decompressor: LayeredPointRecordDecompressor<'a, &'a mut R>,
    record_size: usize,
}

impl<'a, R: Read + Seek + Send> ChunkLazDecoder<'a, R> {
    fn new(source: &'a mut R, laz_vlr: LazVlr) -> Result<Self> {
        let mut decompressor = LayeredPointRecordDecompressor::new(source);
        let record_size = configure_layered_decompressor(&mut decompressor, &laz_vlr)?;
        Ok(Self {
            laz_vlr,
            decompressor,
            record_size,
        })
    }

    fn record_size(&self) -> usize {
        self.record_size
    }

    fn seek_to_chunk(&mut self, offset: u64) -> Result<()> {
        self.decompressor
            .get_mut()
            .seek(SeekFrom::Start(offset))
            .map_err(|e| Error::io("seek COPC point chunk", e))?;
        self.decompressor.reset();
        self.record_size = configure_layered_decompressor(&mut self.decompressor, &self.laz_vlr)?;
        Ok(())
    }

    fn decompress_one(&mut self, out: &mut [u8]) -> Result<()> {
        self.decompressor
            .decompress_next(out)
            .map_err(|e| Error::io("decompress COPC point", e))
    }
}

fn configure_layered_decompressor<R: Read + Seek>(
    decompressor: &mut LayeredPointRecordDecompressor<'_, R>,
    laz_vlr: &LazVlr,
) -> Result<usize> {
    decompressor
        .set_fields_from(laz_vlr.items())
        .map_err(|e| Error::Las(e.to_string()))?;
    let record_size = decompressor.record_size();
    if record_size == 0 {
        return Err(Error::Unsupported(
            "COPC point iteration requires layered LAZ point records".into(),
        ));
    }
    Ok(record_size)
}

fn select_point_chunks(file: &CopcFile, query: PointQuery) -> Result<Vec<Entry>> {
    let (level_min, level_max) = level_range(query.lod, file.copc_info())?;
    let query_bounds = match query.bounds {
        BoundsSelection::All => None,
        BoundsSelection::Within(bounds) => Some(bounds),
    };

    let mut chunks = Vec::new();
    for entry in file.hierarchy_entries() {
        if !entry.has_point_data() {
            continue;
        }
        if entry.byte_size <= 0 {
            return Err(Error::InvalidData(format!(
                "point chunk {:?} has invalid byte size {}",
                entry.key, entry.byte_size
            )));
        }
        if !(level_min..level_max).contains(&entry.key.level) {
            continue;
        }
        if let Some(bounds) = query_bounds {
            let node_bounds = voxel_bounds(entry.key, file.copc_info())?;
            if !node_bounds.intersects(bounds) {
                continue;
            }
        }
        chunks.push(*entry);
    }
    chunks.sort_by_key(|entry| (entry.offset, entry.key));
    Ok(chunks)
}

fn level_range(selection: LodSelection, info: &CopcInfo) -> Result<(i32, i32)> {
    match selection {
        LodSelection::All => Ok((0, i32::MAX)),
        LodSelection::Resolution(resolution) => {
            if !resolution.is_finite() || resolution <= 0.0 {
                return Err(Error::InvalidInput(format!(
                    "resolution must be finite and positive, got {resolution}"
                )));
            }
            if !info.spacing.is_finite() || info.spacing <= 0.0 {
                return Err(Error::InvalidData(format!(
                    "COPC spacing must be finite and positive, got {}",
                    info.spacing
                )));
            }
            let level_max = ((info.spacing / resolution).log2().ceil() as i64 + 1)
                .max(1)
                .min(i64::from(i32::MAX)) as i32;
            Ok((0, level_max))
        }
        LodSelection::Level(level) => {
            validate_level(level)?;
            let max = level
                .checked_add(1)
                .ok_or_else(|| Error::InvalidInput(format!("LOD level {level} is too large")))?;
            Ok((level, max))
        }
        LodSelection::LevelMinMax(min, max) => {
            validate_level(min)?;
            validate_level(max)?;
            if max < min {
                return Err(Error::InvalidInput(format!(
                    "LOD max {max} is smaller than min {min}"
                )));
            }
            Ok((min, max))
        }
    }
}

fn validate_level(level: i32) -> Result<()> {
    if level < 0 {
        return Err(Error::InvalidInput(format!(
            "LOD level must be non-negative, got {level}"
        )));
    }
    Ok(())
}

fn total_candidate_points(entries: &[Entry]) -> Result<usize> {
    entries.iter().try_fold(0usize, |total, entry| {
        let count = usize::try_from(entry.point_count).map_err(|_| {
            Error::InvalidData(format!(
                "negative point count {} for {:?}",
                entry.point_count, entry.key
            ))
        })?;
        total
            .checked_add(count)
            .ok_or_else(|| Error::InvalidData("selected point count overflows usize".into()))
    })
}

fn voxel_bounds(key: copc_core::VoxelKey, info: &CopcInfo) -> Result<Bounds> {
    if key.level < 0 || key.x < 0 || key.y < 0 || key.z < 0 {
        return Err(Error::InvalidData(format!(
            "invalid negative voxel key {:?}",
            key
        )));
    }
    let side = (info.halfsize * 2.0) / 2.0_f64.powi(key.level);
    let root_min = (
        info.center.0 - info.halfsize,
        info.center.1 - info.halfsize,
        info.center.2 - info.halfsize,
    );
    let min = (
        root_min.0 + f64::from(key.x) * side,
        root_min.1 + f64::from(key.y) * side,
        root_min.2 + f64::from(key.z) * side,
    );
    Ok(Bounds::new(min, (min.0 + side, min.1 + side, min.2 + side)))
}
