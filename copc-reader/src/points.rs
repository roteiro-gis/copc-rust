use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;

use copc_core::{
    layout_for_las_format, scan_angle_rank_from_degrees, Bounds, CancelCheck, ColumnData,
    ColumnSelection, ColumnSpec, CopcInfo, Entry, Error, LasColumnBatch, LasDimension, Result,
};
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

    pub fn read_columns(
        &mut self,
        query: PointQuery,
        selection: ColumnSelection,
    ) -> Result<LasColumnBatch> {
        self.read_columns_inner(query, selection, None)
    }

    pub fn read_columns_with_cancel(
        &mut self,
        query: PointQuery,
        selection: ColumnSelection,
        cancel: &dyn CancelCheck,
    ) -> Result<LasColumnBatch> {
        self.read_columns_inner(query, selection, Some(cancel))
    }

    fn read_columns_inner(
        &mut self,
        query: PointQuery,
        selection: ColumnSelection,
        cancel: Option<&dyn CancelCheck>,
    ) -> Result<LasColumnBatch> {
        let chunks = select_point_chunks(&self.file, query)?;
        let point_format = self.file.point_format()?;
        let transforms = self.file.transforms();
        let bounds = match query.bounds {
            BoundsSelection::All => None,
            BoundsSelection::Within(bounds) => Some(bounds),
        };
        let mut decoder = ChunkLazDecoder::new(&mut self.source, self.file.laszip_vlr().clone())?;
        let expected_record_size = usize::from(point_format.len());
        if decoder.record_size() != expected_record_size {
            return Err(Error::InvalidData(format!(
                "LASzip item size is {} bytes, but LAS point record length is {} bytes",
                decoder.record_size(),
                expected_record_size
            )));
        }

        let capacity = match bounds {
            Some(_) => 0,
            None => total_candidate_points(&chunks)?,
        };
        let mut columns = selected_column_builders(point_format, selection, capacity)?;
        let mut point_buf = vec![0u8; expected_record_size];
        let mut decoded_points = 0usize;
        let mut accepted_points = 0usize;

        for entry in chunks {
            if entry.point_count <= 0 {
                continue;
            }
            decoder.seek_to_chunk(entry.offset)?;
            let points_in_chunk = usize::try_from(entry.point_count).map_err(|_| {
                Error::InvalidData(format!(
                    "negative point count {} for {:?}",
                    entry.point_count, entry.key
                ))
            })?;
            for _ in 0..points_in_chunk {
                if decoded_points % CANCEL_POLL_STRIDE == 0 {
                    if let Some(cancel) = cancel {
                        cancel.check()?;
                    }
                }

                decoder.decompress_one(&mut point_buf)?;
                decoded_points += 1;

                let raw_point = las::raw::Point::read_from(point_buf.as_slice(), &point_format)
                    .map_err(|e| Error::Las(e.to_string()))?;
                let x = transforms.x.direct(raw_point.x);
                let y = transforms.y.direct(raw_point.y);
                let z = transforms.z.direct(raw_point.z);
                if let Some(bounds) = bounds {
                    if !bounds.contains_xyz(x, y, z) {
                        continue;
                    }
                }

                append_columns(&mut columns, &raw_point, (x, y, z))?;
                accepted_points += 1;
            }
        }

        let batch = LasColumnBatch {
            len: accepted_points,
            columns,
        };
        batch.validate()?;
        Ok(batch)
    }
}

fn selected_column_builders(
    point_format: LasPointFormat,
    selection: ColumnSelection,
    capacity: usize,
) -> Result<Vec<(ColumnSpec, ColumnData)>> {
    layout_for_las_format(point_format)
        .into_iter()
        .filter(|spec| selection.contains(spec.dimension))
        .map(|spec| empty_column(spec, capacity))
        .collect()
}

fn empty_column(spec: ColumnSpec, capacity: usize) -> Result<(ColumnSpec, ColumnData)> {
    let data = match spec.scalar {
        copc_core::ScalarType::F64 => ColumnData::F64(Vec::with_capacity(capacity)),
        copc_core::ScalarType::F32 => ColumnData::F32(Vec::with_capacity(capacity)),
        copc_core::ScalarType::I64 => ColumnData::I64(Vec::with_capacity(capacity)),
        copc_core::ScalarType::I32 => ColumnData::I32(Vec::with_capacity(capacity)),
        copc_core::ScalarType::I16 => ColumnData::I16(Vec::with_capacity(capacity)),
        copc_core::ScalarType::I8 => ColumnData::I8(Vec::with_capacity(capacity)),
        copc_core::ScalarType::U64 => ColumnData::U64(Vec::with_capacity(capacity)),
        copc_core::ScalarType::U32 => ColumnData::U32(Vec::with_capacity(capacity)),
        copc_core::ScalarType::U16 => ColumnData::U16(Vec::with_capacity(capacity)),
        copc_core::ScalarType::U8 => {
            let capacity = if spec.dimension == LasDimension::ExtraBytes {
                let width = spec.extra_byte_width().ok_or_else(|| {
                    Error::InvalidInput("ExtraBytes column requires a non-zero byte width".into())
                })?;
                capacity.checked_mul(width).ok_or_else(|| {
                    Error::InvalidInput("ExtraBytes column capacity exceeds usize range".into())
                })?
            } else {
                capacity
            };
            ColumnData::U8(Vec::with_capacity(capacity))
        }
        copc_core::ScalarType::Bool => ColumnData::Bool(Vec::with_capacity(capacity)),
    };
    Ok((spec, data))
}

fn append_columns(
    columns: &mut [(ColumnSpec, ColumnData)],
    raw_point: &las::raw::Point,
    xyz: (f64, f64, f64),
) -> Result<()> {
    let mut flags = raw_point.flags;
    let is_overlap = flags.is_overlap();
    flags.clear_overlap_class();
    let classification = u8::from(
        flags
            .to_classification()
            .map_err(|e| Error::Las(e.to_string()))?,
    );
    let scan_direction_flag = matches!(
        flags.scan_direction(),
        las::point::ScanDirection::LeftToRight
    );
    let scan_angle_rank = scan_angle_rank_from_degrees(f32::from(raw_point.scan_angle));
    let context = ColumnAppendContext {
        raw_point,
        xyz,
        flags,
        classification,
        is_overlap,
        scan_direction_flag,
        scan_angle_rank,
    };

    for (spec, data) in columns {
        append_column(*spec, data, &context)?;
    }
    Ok(())
}

struct ColumnAppendContext<'a> {
    raw_point: &'a las::raw::Point,
    xyz: (f64, f64, f64),
    flags: las::raw::point::Flags,
    classification: u8,
    is_overlap: bool,
    scan_direction_flag: bool,
    scan_angle_rank: i16,
}

fn append_column(
    spec: ColumnSpec,
    data: &mut ColumnData,
    context: &ColumnAppendContext<'_>,
) -> Result<()> {
    let dimension = spec.dimension;
    let scalar = data.scalar();
    match (dimension, data) {
        (LasDimension::X, ColumnData::F64(values)) => values.push(context.xyz.0),
        (LasDimension::Y, ColumnData::F64(values)) => values.push(context.xyz.1),
        (LasDimension::Z, ColumnData::F64(values)) => values.push(context.xyz.2),
        (LasDimension::Intensity, ColumnData::U16(values)) => {
            values.push(context.raw_point.intensity);
        }
        (LasDimension::ReturnNumber, ColumnData::U8(values)) => {
            values.push(context.flags.return_number());
        }
        (LasDimension::NumberOfReturns, ColumnData::U8(values)) => {
            values.push(context.flags.number_of_returns());
        }
        (LasDimension::Classification, ColumnData::U8(values)) => {
            values.push(context.classification);
        }
        (LasDimension::ScanDirectionFlag, ColumnData::Bool(values)) => {
            values.push(context.scan_direction_flag);
        }
        (LasDimension::EdgeOfFlightLine, ColumnData::Bool(values)) => {
            values.push(context.flags.is_edge_of_flight_line());
        }
        (LasDimension::ScanAngleRank, ColumnData::I16(values)) => {
            values.push(context.scan_angle_rank);
        }
        (LasDimension::UserData, ColumnData::U8(values)) => {
            values.push(context.raw_point.user_data);
        }
        (LasDimension::PointSourceId, ColumnData::U16(values)) => {
            values.push(context.raw_point.point_source_id);
        }
        (LasDimension::Synthetic, ColumnData::Bool(values)) => {
            values.push(context.flags.is_synthetic());
        }
        (LasDimension::KeyPoint, ColumnData::Bool(values)) => {
            values.push(context.flags.is_key_point());
        }
        (LasDimension::Withheld, ColumnData::Bool(values)) => {
            values.push(context.flags.is_withheld());
        }
        (LasDimension::Overlap, ColumnData::Bool(values)) => values.push(context.is_overlap),
        (LasDimension::ScanChannel, ColumnData::U8(values)) => {
            values.push(context.flags.scanner_channel());
        }
        (LasDimension::GpsTime, ColumnData::F64(values)) => {
            values.push(context.raw_point.gps_time.unwrap_or(0.0));
        }
        (LasDimension::Red, ColumnData::U16(values)) => {
            values.push(context.raw_point.color.unwrap_or_default().red);
        }
        (LasDimension::Green, ColumnData::U16(values)) => {
            values.push(context.raw_point.color.unwrap_or_default().green);
        }
        (LasDimension::Blue, ColumnData::U16(values)) => {
            values.push(context.raw_point.color.unwrap_or_default().blue);
        }
        (LasDimension::Nir, ColumnData::U16(values)) => {
            values.push(context.raw_point.nir.unwrap_or(0));
        }
        (LasDimension::WaveformPacketDescriptorIndex, ColumnData::U8(values)) => {
            values.push(
                context
                    .raw_point
                    .waveform
                    .unwrap_or_default()
                    .wave_packet_descriptor_index,
            );
        }
        (LasDimension::WaveformPacketByteOffset, ColumnData::U64(values)) => {
            values.push(
                context
                    .raw_point
                    .waveform
                    .unwrap_or_default()
                    .byte_offset_to_waveform_data,
            );
        }
        (LasDimension::WaveformPacketSize, ColumnData::U32(values)) => {
            values.push(
                context
                    .raw_point
                    .waveform
                    .unwrap_or_default()
                    .waveform_packet_size_in_bytes,
            );
        }
        (LasDimension::WavePacketReturnPointWaveformLocation, ColumnData::F32(values)) => {
            values.push(
                context
                    .raw_point
                    .waveform
                    .unwrap_or_default()
                    .return_point_waveform_location,
            );
        }
        (LasDimension::ExtraBytes, ColumnData::U8(values)) => {
            let width = spec.extra_byte_width().ok_or_else(|| {
                Error::InvalidData("ExtraBytes column requires a non-zero byte width".into())
            })?;
            if context.raw_point.extra_bytes.len() != width {
                return Err(Error::InvalidData(format!(
                    "ExtraBytes point has {} bytes, expected {width}",
                    context.raw_point.extra_bytes.len()
                )));
            }
            values.extend_from_slice(&context.raw_point.extra_bytes);
        }
        _ => {
            return Err(Error::InvalidData(format!(
                "column {:?} has incompatible data type {:?}",
                dimension, scalar
            )));
        }
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_column_builders_include_extra_bytes_width() {
        let mut format = LasPointFormat::new(6).unwrap();
        format.extra_bytes = 3;

        let columns = selected_column_builders(format, ColumnSelection::all(), 2).unwrap();

        let extra_spec = columns
            .iter()
            .map(|(spec, _)| *spec)
            .find(|spec| spec.dimension == LasDimension::ExtraBytes)
            .expect("ExtraBytes column spec");
        assert_eq!(Some(3), extra_spec.extra_byte_width());
        assert_eq!(copc_core::ScalarType::U8, extra_spec.scalar);
    }

    #[test]
    fn append_columns_preserves_fixed_width_extra_bytes() {
        let mut format = LasPointFormat::new(6).unwrap();
        format.extra_bytes = 3;
        let mut columns = selected_column_builders(
            format,
            ColumnSelection::from_dimensions([LasDimension::X, LasDimension::ExtraBytes]),
            1,
        )
        .unwrap();
        let raw_point = las::raw::Point {
            x: 10,
            y: 20,
            z: 30,
            flags: las::raw::point::Flags::ThreeByte(1 | (1 << 4), 0, 2),
            scan_angle: las::raw::point::ScanAngle::from(0.0),
            extra_bytes: vec![9, 8, 7],
            ..Default::default()
        };

        append_columns(&mut columns, &raw_point, (1.0, 2.0, 3.0)).unwrap();
        let batch = LasColumnBatch::new(columns).unwrap();

        assert_eq!(1, batch.len());
        assert_eq!(
            Some(&ColumnData::U8(vec![9, 8, 7])),
            batch.column(LasDimension::ExtraBytes)
        );
    }

    #[test]
    fn append_columns_rejects_wrong_extra_bytes_width() {
        let mut format = LasPointFormat::new(6).unwrap();
        format.extra_bytes = 3;
        let mut columns = selected_column_builders(
            format,
            ColumnSelection::from_dimensions([LasDimension::ExtraBytes]),
            1,
        )
        .unwrap();
        let raw_point = las::raw::Point {
            flags: las::raw::point::Flags::ThreeByte(1 | (1 << 4), 0, 2),
            extra_bytes: vec![9, 8],
            ..Default::default()
        };

        let err = append_columns(&mut columns, &raw_point, (1.0, 2.0, 3.0)).unwrap_err();

        assert!(err.to_string().contains("expected 3"));
    }
}
