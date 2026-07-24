use std::fs::File;
use std::io::{BufReader, Cursor, Read, Seek, SeekFrom};
use std::path::Path;

use copc_core::{
    layout_for_las_format, Bounds, CancelCheck, ColumnData, ColumnSelection, ColumnSpec, CopcInfo,
    Entry, Error, LasColumnBatch, LasDimension, Result,
};
use las::point::Format as LasPointFormat;
use las::{Point, Transform, Vector};
use laz::record::{LayeredPointRecordDecompressor, RecordDecompressor};
use laz::LazVlr;

use crate::{CopcFile, LasHeader};

const CANCEL_POLL_STRIDE: usize = 4_096;
/// Upper bound on the column capacity reserved up front from hierarchy point
/// counts. Counts are validated against the LAS header total at open, but the
/// header itself is untrusted input, so reservations beyond this grow lazily.
pub(crate) const MAX_INITIAL_COLUMN_RESERVE_POINTS: usize = 4 * 1024 * 1024;

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
        let decoder = ChunkColumnDecoder::new(
            self.file.laszip_vlr().clone(),
            point_format,
            self.file.transforms(),
            match query.bounds {
                BoundsSelection::All => None,
                BoundsSelection::Within(bounds) => Some(bounds),
            },
        )?;

        let capacity = match decoder.bounds {
            Some(_) => 0,
            None => total_candidate_points(&chunks)?.min(MAX_INITIAL_COLUMN_RESERVE_POINTS),
        };
        let mut columns = selected_column_builders(point_format, selection.clone(), capacity)?;
        let mut accepted_points = 0usize;

        #[cfg(not(feature = "parallel"))]
        {
            let mut chunk_bytes = Vec::new();
            for entry in chunks {
                if let Some(cancel) = cancel {
                    cancel.check()?;
                }
                let points_in_chunk = read_chunk_bytes(&mut self.source, entry, &mut chunk_bytes)?;
                accepted_points +=
                    decoder.decode_into(&chunk_bytes, points_in_chunk, &mut columns, cancel)?;
            }
        }

        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;

            type DecodedChunk = Result<(Vec<(ColumnSpec, ColumnData)>, usize)>;

            let batch_size = rayon::current_num_threads().max(1) * 2;
            for batch in chunks.chunks(batch_size) {
                if let Some(cancel) = cancel {
                    cancel.check()?;
                }
                let mut batch_bytes = Vec::with_capacity(batch.len());
                for entry in batch {
                    let mut chunk_bytes = Vec::new();
                    let points_in_chunk =
                        read_chunk_bytes(&mut self.source, *entry, &mut chunk_bytes)?;
                    batch_bytes.push((chunk_bytes, points_in_chunk));
                }
                let decoded: Vec<DecodedChunk> = batch_bytes
                    .par_iter()
                    .map(|(chunk_bytes, points_in_chunk)| {
                        let mut chunk_columns = selected_column_builders(
                            point_format,
                            selection.clone(),
                            (*points_in_chunk).min(MAX_INITIAL_COLUMN_RESERVE_POINTS),
                        )?;
                        let accepted = decoder.decode_into(
                            chunk_bytes,
                            *points_in_chunk,
                            &mut chunk_columns,
                            None,
                        )?;
                        Ok((chunk_columns, accepted))
                    })
                    .collect();
                for result in decoded {
                    let (chunk_columns, accepted) = result?;
                    merge_columns(&mut columns, chunk_columns)?;
                    accepted_points += accepted;
                }
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

/// Reads one chunk's compressed bytes into `chunk_bytes` (replacing its
/// contents) and returns the chunk's point count.
fn read_chunk_bytes<R: Read + Seek>(
    source: &mut R,
    entry: Entry,
    chunk_bytes: &mut Vec<u8>,
) -> Result<usize> {
    let points_in_chunk = usize::try_from(entry.point_count).map_err(|_| {
        Error::InvalidData(format!(
            "negative point count {} for {:?}",
            entry.point_count, entry.key
        ))
    })?;
    let byte_size = usize::try_from(entry.byte_size).map_err(|_| {
        Error::InvalidData(format!(
            "invalid byte size {} for {:?}",
            entry.byte_size, entry.key
        ))
    })?;
    chunk_bytes.clear();
    chunk_bytes.resize(byte_size, 0);
    source
        .seek(SeekFrom::Start(entry.offset))
        .map_err(|e| Error::io("seek COPC point chunk", e))?;
    source
        .read_exact(chunk_bytes)
        .map_err(|e| Error::io("read COPC point chunk", e))?;
    Ok(points_in_chunk)
}

/// Decodes standalone COPC chunk bytes into column builders.
pub(crate) struct ChunkColumnDecoder {
    laz_vlr: LazVlr,
    layout: ExtendedPointLayout,
    transforms: Vector<Transform>,
    bounds: Option<Bounds>,
    record_size: usize,
}

impl ChunkColumnDecoder {
    pub(crate) fn new(
        laz_vlr: LazVlr,
        point_format: LasPointFormat,
        transforms: Vector<Transform>,
        bounds: Option<Bounds>,
    ) -> Result<Self> {
        let layout = ExtendedPointLayout::for_format(&point_format)?;
        let record_size = validated_record_size(&laz_vlr, &point_format)?;
        Ok(Self {
            laz_vlr,
            layout,
            transforms,
            bounds,
            record_size,
        })
    }

    /// Decode `points_in_chunk` points from one chunk's compressed bytes,
    /// appending selected values to `columns`; returns the number of points
    /// accepted by the bounds filter.
    pub(crate) fn decode_into(
        &self,
        chunk_bytes: &[u8],
        points_in_chunk: usize,
        columns: &mut [(ColumnSpec, ColumnData)],
        cancel: Option<&dyn CancelCheck>,
    ) -> Result<usize> {
        let mut decompressor = LayeredPointRecordDecompressor::new(Cursor::new(chunk_bytes));
        configure_layered_decompressor(&mut decompressor, &self.laz_vlr)?;
        let mut point_buf = vec![0u8; self.record_size];
        let mut accepted = 0usize;
        for decoded in 0..points_in_chunk {
            if decoded % CANCEL_POLL_STRIDE == 0 {
                if let Some(cancel) = cancel {
                    cancel.check()?;
                }
            }
            decompressor
                .decompress_next(&mut point_buf)
                .map_err(|e| Error::io("decompress COPC point", e))?;

            let x = self.transforms.x.direct(i32_at(&point_buf, 0));
            let y = self.transforms.y.direct(i32_at(&point_buf, 4));
            let z = self.transforms.z.direct(i32_at(&point_buf, 8));
            if let Some(bounds) = self.bounds {
                if !bounds.contains_xyz(x, y, z) {
                    continue;
                }
            }

            append_columns(columns, &point_buf, &self.layout, (x, y, z))?;
            accepted += 1;
        }
        Ok(accepted)
    }
}

/// Decodes one chunk's compressed bytes into `las::Point` rows, appending
/// bounds-accepted points to `out`.
pub(crate) fn decode_chunk_points(
    chunk_bytes: &[u8],
    points_in_chunk: usize,
    laz_vlr: &LazVlr,
    point_format: &LasPointFormat,
    transforms: &Vector<Transform>,
    bounds: Option<Bounds>,
    out: &mut Vec<Point>,
) -> Result<()> {
    let mut decompressor = LayeredPointRecordDecompressor::new(Cursor::new(chunk_bytes));
    let record_size = configure_layered_decompressor(&mut decompressor, laz_vlr)?;
    if record_size != usize::from(point_format.len()) {
        return Err(Error::InvalidData(format!(
            "LASzip item size is {record_size} bytes, but LAS point record length is {} bytes",
            point_format.len()
        )));
    }
    let mut point_buf = vec![0u8; record_size];
    for _ in 0..points_in_chunk {
        decompressor
            .decompress_next(&mut point_buf)
            .map_err(|e| Error::io("decompress COPC point", e))?;
        let raw_point = las::raw::Point::read_from(point_buf.as_slice(), point_format)
            .map_err(|e| Error::Las(e.to_string()))?;
        if let Some(bounds) = bounds {
            let x = transforms.x.direct(raw_point.x);
            let y = transforms.y.direct(raw_point.y);
            let z = transforms.z.direct(raw_point.z);
            if !bounds.contains_xyz(x, y, z) {
                continue;
            }
        }
        out.push(Point::new(raw_point, transforms));
    }
    Ok(())
}

/// Validates the LASzip item layout against the LAS point record length and
/// returns the record size.
fn validated_record_size(laz_vlr: &LazVlr, point_format: &LasPointFormat) -> Result<usize> {
    let mut probe = LayeredPointRecordDecompressor::new(Cursor::new(&[][..]));
    let record_size = configure_layered_decompressor(&mut probe, laz_vlr)?;
    let expected = usize::from(point_format.len());
    if record_size != expected {
        return Err(Error::InvalidData(format!(
            "LASzip item size is {record_size} bytes, but LAS point record length is {expected} bytes"
        )));
    }
    Ok(expected)
}

/// Appends `src` column data onto `dst`, preserving chunk order.
#[cfg(feature = "parallel")]
fn merge_columns(
    dst: &mut [(ColumnSpec, ColumnData)],
    src: Vec<(ColumnSpec, ColumnData)>,
) -> Result<()> {
    if dst.len() != src.len() {
        return Err(Error::InvalidData(format!(
            "chunk produced {} columns, expected {}",
            src.len(),
            dst.len()
        )));
    }
    for ((dst_spec, dst_data), (src_spec, src_data)) in dst.iter_mut().zip(src) {
        if *dst_spec != src_spec {
            return Err(Error::InvalidData(format!(
                "chunk column spec mismatch: found {src_spec:?}, expected {dst_spec:?}"
            )));
        }
        match (dst_data, src_data) {
            (ColumnData::F64(dst), ColumnData::F64(src)) => dst.extend_from_slice(&src),
            (ColumnData::F32(dst), ColumnData::F32(src)) => dst.extend_from_slice(&src),
            (ColumnData::I64(dst), ColumnData::I64(src)) => dst.extend_from_slice(&src),
            (ColumnData::I32(dst), ColumnData::I32(src)) => dst.extend_from_slice(&src),
            (ColumnData::I16(dst), ColumnData::I16(src)) => dst.extend_from_slice(&src),
            (ColumnData::I8(dst), ColumnData::I8(src)) => dst.extend_from_slice(&src),
            (ColumnData::U64(dst), ColumnData::U64(src)) => dst.extend_from_slice(&src),
            (ColumnData::U32(dst), ColumnData::U32(src)) => dst.extend_from_slice(&src),
            (ColumnData::U16(dst), ColumnData::U16(src)) => dst.extend_from_slice(&src),
            (ColumnData::U8(dst), ColumnData::U8(src)) => dst.extend_from_slice(&src),
            (ColumnData::Bool(dst), ColumnData::Bool(src)) => dst.extend_from_slice(&src),
            _ => {
                return Err(Error::InvalidData(
                    "chunk column scalar mismatch during merge".into(),
                ))
            }
        }
    }
    Ok(())
}

pub(crate) fn selected_column_builders(
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

/// Reader-side scan angle scale, matching `las::raw::point::ScanAngle`'s
/// scaled-to-degrees conversion for LAS 1.4 extended formats.
const LAS_14_SCAN_ANGLE_SCALE: f32 = 0.006;
/// Overlap classification code; extended points with this class are reported
/// as `Overlap = true`, `Classification = 1`, matching `las::raw::point::Flags`.
const OVERLAP_CLASSIFICATION_CODE: u8 = 12;

/// Field offsets within an extended (PDRF 6-10) point record, so columns can
/// be filled directly from the decompressed record bytes without building a
/// `las::raw::Point` (and its `extra_bytes` allocation) per point.
struct ExtendedPointLayout {
    color_offset: Option<usize>,
    nir_offset: Option<usize>,
    waveform_offset: Option<usize>,
    extra_offset: usize,
    extra_len: usize,
}

impl ExtendedPointLayout {
    fn for_format(format: &LasPointFormat) -> Result<Self> {
        if !format.is_extended {
            return Err(Error::Unsupported(
                "COPC column reads require extended point formats (6-10)".into(),
            ));
        }
        let mut cursor = 30usize;
        let color_offset = if format.has_color {
            let offset = cursor;
            cursor += 6;
            Some(offset)
        } else {
            None
        };
        let nir_offset = if format.has_nir {
            let offset = cursor;
            cursor += 2;
            Some(offset)
        } else {
            None
        };
        let waveform_offset = if format.has_waveform {
            let offset = cursor;
            cursor += 29;
            Some(offset)
        } else {
            None
        };
        let extra_offset = cursor;
        let extra_len = usize::from(format.extra_bytes);
        debug_assert_eq!(extra_offset + extra_len, usize::from(format.len()));
        Ok(Self {
            color_offset,
            nir_offset,
            waveform_offset,
            extra_offset,
            extra_len,
        })
    }
}

fn append_columns(
    columns: &mut [(ColumnSpec, ColumnData)],
    buf: &[u8],
    layout: &ExtendedPointLayout,
    xyz: (f64, f64, f64),
) -> Result<()> {
    let raw_class = buf[16];
    let context = ColumnAppendContext {
        buf,
        layout,
        xyz,
        classification: if raw_class == OVERLAP_CLASSIFICATION_CODE {
            1
        } else {
            raw_class
        },
        is_overlap: buf[15] & 8 == 8,
    };

    for (spec, data) in columns {
        append_column(*spec, data, &context)?;
    }
    Ok(())
}

struct ColumnAppendContext<'a> {
    buf: &'a [u8],
    layout: &'a ExtendedPointLayout,
    xyz: (f64, f64, f64),
    classification: u8,
    is_overlap: bool,
}

#[inline]
fn i32_at(buf: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(buf[offset..offset + 4].try_into().expect("i32 width"))
}

#[inline]
fn u16_at(buf: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(buf[offset..offset + 2].try_into().expect("u16 width"))
}

#[inline]
fn u32_at(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(buf[offset..offset + 4].try_into().expect("u32 width"))
}

#[inline]
fn u64_at(buf: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(buf[offset..offset + 8].try_into().expect("u64 width"))
}

#[inline]
fn f32_at(buf: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(buf[offset..offset + 4].try_into().expect("f32 width"))
}

#[inline]
fn f64_at(buf: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes(buf[offset..offset + 8].try_into().expect("f64 width"))
}

#[inline]
fn i16_at(buf: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes(buf[offset..offset + 2].try_into().expect("i16 width"))
}

fn append_column(
    spec: ColumnSpec,
    data: &mut ColumnData,
    context: &ColumnAppendContext<'_>,
) -> Result<()> {
    let dimension = spec.dimension;
    let scalar = data.scalar();
    let buf = context.buf;
    let layout = context.layout;
    match (dimension, data) {
        (LasDimension::X, ColumnData::F64(values)) => values.push(context.xyz.0),
        (LasDimension::Y, ColumnData::F64(values)) => values.push(context.xyz.1),
        (LasDimension::Z, ColumnData::F64(values)) => values.push(context.xyz.2),
        (LasDimension::Intensity, ColumnData::U16(values)) => {
            values.push(u16_at(buf, 12));
        }
        (LasDimension::ReturnNumber, ColumnData::U8(values)) => {
            values.push(buf[14] & 15);
        }
        (LasDimension::NumberOfReturns, ColumnData::U8(values)) => {
            values.push((buf[14] >> 4) & 15);
        }
        (LasDimension::Classification, ColumnData::U8(values)) => {
            values.push(context.classification);
        }
        (LasDimension::ScanDirectionFlag, ColumnData::Bool(values)) => {
            values.push((buf[15] >> 6) & 1 == 1);
        }
        (LasDimension::EdgeOfFlightLine, ColumnData::Bool(values)) => {
            values.push((buf[15] >> 7) == 1);
        }
        (LasDimension::ScanAngle, ColumnData::F32(values)) => {
            values.push(f32::from(i16_at(buf, 18)) * LAS_14_SCAN_ANGLE_SCALE);
        }
        (LasDimension::UserData, ColumnData::U8(values)) => {
            values.push(buf[17]);
        }
        (LasDimension::PointSourceId, ColumnData::U16(values)) => {
            values.push(u16_at(buf, 20));
        }
        (LasDimension::Synthetic, ColumnData::Bool(values)) => {
            values.push(buf[15] & 1 == 1);
        }
        (LasDimension::KeyPoint, ColumnData::Bool(values)) => {
            values.push(buf[15] & 2 == 2);
        }
        (LasDimension::Withheld, ColumnData::Bool(values)) => {
            values.push(buf[15] & 4 == 4);
        }
        (LasDimension::Overlap, ColumnData::Bool(values)) => values.push(context.is_overlap),
        (LasDimension::ScanChannel, ColumnData::U8(values)) => {
            values.push((buf[15] >> 4) & 3);
        }
        (LasDimension::GpsTime, ColumnData::F64(values)) => {
            values.push(f64_at(buf, 22));
        }
        (LasDimension::Red, ColumnData::U16(values)) => {
            values.push(layout.color_offset.map_or(0, |o| u16_at(buf, o)));
        }
        (LasDimension::Green, ColumnData::U16(values)) => {
            values.push(layout.color_offset.map_or(0, |o| u16_at(buf, o + 2)));
        }
        (LasDimension::Blue, ColumnData::U16(values)) => {
            values.push(layout.color_offset.map_or(0, |o| u16_at(buf, o + 4)));
        }
        (LasDimension::Nir, ColumnData::U16(values)) => {
            values.push(layout.nir_offset.map_or(0, |o| u16_at(buf, o)));
        }
        (LasDimension::WaveformPacketDescriptorIndex, ColumnData::U8(values)) => {
            values.push(layout.waveform_offset.map_or(0, |o| buf[o]));
        }
        (LasDimension::WaveformPacketByteOffset, ColumnData::U64(values)) => {
            values.push(layout.waveform_offset.map_or(0, |o| u64_at(buf, o + 1)));
        }
        (LasDimension::WaveformPacketSize, ColumnData::U32(values)) => {
            values.push(layout.waveform_offset.map_or(0, |o| u32_at(buf, o + 9)));
        }
        (LasDimension::WavePacketReturnPointWaveformLocation, ColumnData::F32(values)) => {
            values.push(layout.waveform_offset.map_or(0.0, |o| f32_at(buf, o + 13)));
        }
        (LasDimension::ExtraBytes, ColumnData::U8(values)) => {
            let width = spec.extra_byte_width().ok_or_else(|| {
                Error::InvalidData("ExtraBytes column requires a non-zero byte width".into())
            })?;
            if layout.extra_len != width {
                return Err(Error::InvalidData(format!(
                    "ExtraBytes point has {} bytes, expected {width}",
                    layout.extra_len
                )));
            }
            values.extend_from_slice(&buf[layout.extra_offset..layout.extra_offset + width]);
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
        let QuerySetup {
            chunks,
            point_format,
            transforms,
            bounds,
            decoder,
            record_size,
        } = QuerySetup::new(source, file, query)?;
        let remaining_candidate_points = total_candidate_points(&chunks)?;
        let point_buf = vec![0u8; record_size];
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
            let byte_size = u64::try_from(entry.byte_size).map_err(|_| {
                Error::InvalidData(format!(
                    "negative byte size {} for {:?}",
                    entry.byte_size, entry.key
                ))
            })?;
            self.decoder.seek_to_chunk(entry.offset, byte_size)?;
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

/// Shared setup for chunked point queries: selected chunks, format metadata,
/// and a configured chunk decoder validated against the LAS record length.
struct QuerySetup<'a, R: Read + Seek + Send> {
    chunks: Vec<Entry>,
    point_format: LasPointFormat,
    transforms: Vector<Transform>,
    bounds: Option<Bounds>,
    decoder: ChunkLazDecoder<'a, R>,
    record_size: usize,
}

impl<'a, R: Read + Seek + Send> QuerySetup<'a, R> {
    fn new(source: &'a mut R, file: &CopcFile, query: PointQuery) -> Result<Self> {
        let chunks = select_point_chunks(file, query)?;
        let point_format = file.point_format()?;
        let transforms = file.transforms();
        let bounds = match query.bounds {
            BoundsSelection::All => None,
            BoundsSelection::Within(bounds) => Some(bounds),
        };
        let decoder = ChunkLazDecoder::new(source, file.laszip_vlr().clone())?;
        let record_size = usize::from(point_format.len());
        if decoder.record_size() != record_size {
            return Err(Error::InvalidData(format!(
                "LASzip item size is {} bytes, but LAS point record length is {} bytes",
                decoder.record_size(),
                record_size
            )));
        }
        Ok(Self {
            chunks,
            point_format,
            transforms,
            bounds,
            decoder,
            record_size,
        })
    }
}

struct ChunkLazDecoder<'a, R: Read + Seek + Send> {
    laz_vlr: LazVlr,
    decompressor: LayeredPointRecordDecompressor<'a, ChunkReadWindow<'a, R>>,
    record_size: usize,
}

impl<'a, R: Read + Seek + Send> ChunkLazDecoder<'a, R> {
    fn new(source: &'a mut R, laz_vlr: LazVlr) -> Result<Self> {
        let mut decompressor = LayeredPointRecordDecompressor::new(ChunkReadWindow::new(source));
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

    fn seek_to_chunk(&mut self, offset: u64, byte_size: u64) -> Result<()> {
        self.decompressor.get_mut().set_range(offset, byte_size)?;
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

/// Restricts the streaming LAZ decoder to the hierarchy entry's declared
/// compressed byte range. Without this window, malformed chunk metadata can
/// make the decoder consume bytes from the following chunk or hierarchy.
struct ChunkReadWindow<'a, R> {
    source: &'a mut R,
    start: u64,
    end: u64,
    position: u64,
}

impl<'a, R: Read + Seek> ChunkReadWindow<'a, R> {
    fn new(source: &'a mut R) -> Self {
        Self {
            source,
            start: 0,
            end: 0,
            position: 0,
        }
    }

    fn set_range(&mut self, start: u64, byte_size: u64) -> Result<()> {
        let end = start
            .checked_add(byte_size)
            .ok_or_else(|| Error::InvalidData("COPC point chunk range overflows u64".into()))?;
        self.source
            .seek(SeekFrom::Start(start))
            .map_err(|e| Error::io("seek COPC point chunk", e))?;
        self.start = start;
        self.end = end;
        self.position = start;
        Ok(())
    }
}

impl<R: Read + Seek> Read for ChunkReadWindow<'_, R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let remaining = self.end.saturating_sub(self.position);
        if remaining == 0 {
            return Ok(0);
        }
        let allowed = usize::try_from(remaining)
            .unwrap_or(usize::MAX)
            .min(buf.len());
        let read = self.source.read(&mut buf[..allowed])?;
        self.position = self
            .position
            .checked_add(read as u64)
            .ok_or_else(|| std::io::Error::other("chunk read position overflow"))?;
        Ok(read)
    }
}

impl<R: Read + Seek> Seek for ChunkReadWindow<'_, R> {
    fn seek(&mut self, position: SeekFrom) -> std::io::Result<u64> {
        let target = match position {
            SeekFrom::Start(offset) => Some(offset),
            SeekFrom::Current(delta) => self.position.checked_add_signed(delta),
            SeekFrom::End(delta) => self.end.checked_add_signed(delta),
        }
        .filter(|target| (self.start..=self.end).contains(target))
        .ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "seek outside COPC point chunk",
            )
        })?;
        self.source.seek(SeekFrom::Start(target))?;
        self.position = target;
        Ok(target)
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
    select_point_chunks_from(file.hierarchy_entries(), file.copc_info(), query)
}

pub(crate) fn select_point_chunks_from<'a, I>(
    entries: I,
    info: &CopcInfo,
    query: PointQuery,
) -> Result<Vec<Entry>>
where
    I: IntoIterator<Item = &'a Entry>,
{
    validate_query(query)?;
    let (level_min, level_max) = level_range(query.lod, info)?;
    let query_bounds = match query.bounds {
        BoundsSelection::All => None,
        BoundsSelection::Within(bounds) => Some(bounds),
    };

    let mut chunks = Vec::new();
    for entry in entries {
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
            let node_bounds = voxel_bounds(entry.key, info)?;
            if !node_bounds.intersects(bounds) {
                continue;
            }
        }
        chunks.push(*entry);
    }
    chunks.sort_by_key(|entry| (entry.offset, entry.key));
    Ok(chunks)
}

pub(crate) fn validate_query(query: PointQuery) -> Result<()> {
    if let BoundsSelection::Within(bounds) = query.bounds {
        for (name, value) in [
            ("min x", bounds.min.0),
            ("min y", bounds.min.1),
            ("min z", bounds.min.2),
            ("max x", bounds.max.0),
            ("max y", bounds.max.1),
            ("max z", bounds.max.2),
        ] {
            if !value.is_finite() {
                return Err(Error::InvalidInput(format!(
                    "query bounds {name} must be finite, got {value}"
                )));
            }
        }
        for (axis, min, max) in [
            ("x", bounds.min.0, bounds.max.0),
            ("y", bounds.min.1, bounds.max.1),
            ("z", bounds.min.2, bounds.max.2),
        ] {
            if min > max {
                return Err(Error::InvalidInput(format!(
                    "query bounds {axis} minimum {min} exceeds maximum {max}"
                )));
            }
        }
    }
    Ok(())
}

pub(crate) fn level_range(selection: LodSelection, info: &CopcInfo) -> Result<(i32, i32)> {
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

pub(crate) fn total_candidate_points(entries: &[Entry]) -> Result<usize> {
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

pub(crate) fn voxel_bounds(key: copc_core::VoxelKey, info: &CopcInfo) -> Result<Bounds> {
    key.validate()?;
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

    fn record_bytes(format: &LasPointFormat, raw_point: &las::raw::Point) -> Vec<u8> {
        let mut buf = Vec::with_capacity(usize::from(format.len()));
        raw_point.write_to(&mut buf, format).unwrap();
        buf
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
        let buf = record_bytes(&format, &raw_point);
        let layout = ExtendedPointLayout::for_format(&format).unwrap();

        append_columns(&mut columns, &buf, &layout, (1.0, 2.0, 3.0)).unwrap();
        let batch = LasColumnBatch::new(columns).unwrap();

        assert_eq!(1, batch.len());
        assert_eq!(
            Some(&ColumnData::U8(vec![9, 8, 7])),
            batch.column(LasDimension::ExtraBytes)
        );
    }

    #[test]
    fn append_columns_rejects_layout_and_spec_width_mismatch() {
        // Spec widths come from the same format as the layout, so a mismatch
        // is unreachable through the public API; guard the internal error.
        let mut spec_format = LasPointFormat::new(6).unwrap();
        spec_format.extra_bytes = 3;
        let mut columns = selected_column_builders(
            spec_format,
            ColumnSelection::from_dimensions([LasDimension::ExtraBytes]),
            1,
        )
        .unwrap();
        let mut buf_format = LasPointFormat::new(6).unwrap();
        buf_format.extra_bytes = 2;
        let raw_point = las::raw::Point {
            flags: las::raw::point::Flags::ThreeByte(1 | (1 << 4), 0, 2),
            extra_bytes: vec![9, 8],
            ..Default::default()
        };
        let buf = record_bytes(&buf_format, &raw_point);
        let layout = ExtendedPointLayout::for_format(&buf_format).unwrap();

        let err = append_columns(&mut columns, &buf, &layout, (1.0, 2.0, 3.0)).unwrap_err();

        assert!(err.to_string().contains("expected 3"));
    }

    /// Every column decoded directly from record bytes must match the values
    /// the `las` crate parses from the same bytes.
    #[test]
    fn direct_column_decode_matches_las_raw_point() {
        for format_id in [6u8, 7, 8] {
            let mut format = LasPointFormat::new(format_id).unwrap();
            format.extra_bytes = 2;
            let raw_point = las::raw::Point {
                x: -1234,
                y: 5678,
                z: 91011,
                intensity: 0xBEEF,
                flags: las::raw::point::Flags::ThreeByte(
                    3 | (5 << 4),
                    1 | (1 << 2) | (2 << 4) | (1 << 6) | (1 << 7),
                    6,
                ),
                scan_angle: las::raw::point::ScanAngle::Scaled(-5042),
                user_data: 0x42,
                point_source_id: 0xCAFE,
                gps_time: Some(1.234e9),
                color: format
                    .has_color
                    .then_some(las::Color::new(1000, 2000, 3000)),
                nir: format.has_nir.then_some(0xCDCD),
                waveform: None,
                extra_bytes: vec![7, 9],
            };
            let buf = record_bytes(&format, &raw_point);
            let layout = ExtendedPointLayout::for_format(&format).unwrap();
            let mut columns = selected_column_builders(format, ColumnSelection::all(), 1).unwrap();

            append_columns(&mut columns, &buf, &layout, (1.0, 2.0, 3.0)).unwrap();
            let batch = LasColumnBatch::new(columns).unwrap();

            let mut flags = raw_point.flags;
            let expected_overlap = flags.is_overlap();
            flags.clear_overlap_class();
            assert_eq!(
                Some(&ColumnData::U16(vec![raw_point.intensity])),
                batch.column(LasDimension::Intensity)
            );
            assert_eq!(
                Some(&ColumnData::U8(vec![flags.return_number()])),
                batch.column(LasDimension::ReturnNumber)
            );
            assert_eq!(
                Some(&ColumnData::U8(vec![flags.number_of_returns()])),
                batch.column(LasDimension::NumberOfReturns)
            );
            assert_eq!(
                Some(&ColumnData::U8(vec![u8::from(
                    flags.to_classification().unwrap()
                )])),
                batch.column(LasDimension::Classification)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![matches!(
                    flags.scan_direction(),
                    las::point::ScanDirection::LeftToRight
                )])),
                batch.column(LasDimension::ScanDirectionFlag)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![flags.is_edge_of_flight_line()])),
                batch.column(LasDimension::EdgeOfFlightLine)
            );
            assert_eq!(
                Some(&ColumnData::F32(vec![f32::from(raw_point.scan_angle)])),
                batch.column(LasDimension::ScanAngle)
            );
            assert_eq!(
                Some(&ColumnData::U8(vec![raw_point.user_data])),
                batch.column(LasDimension::UserData)
            );
            assert_eq!(
                Some(&ColumnData::U16(vec![raw_point.point_source_id])),
                batch.column(LasDimension::PointSourceId)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![flags.is_synthetic()])),
                batch.column(LasDimension::Synthetic)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![flags.is_key_point()])),
                batch.column(LasDimension::KeyPoint)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![flags.is_withheld()])),
                batch.column(LasDimension::Withheld)
            );
            assert_eq!(
                Some(&ColumnData::Bool(vec![expected_overlap])),
                batch.column(LasDimension::Overlap)
            );
            assert_eq!(
                Some(&ColumnData::U8(vec![flags.scanner_channel()])),
                batch.column(LasDimension::ScanChannel)
            );
            assert_eq!(
                Some(&ColumnData::F64(vec![raw_point.gps_time.unwrap()])),
                batch.column(LasDimension::GpsTime)
            );
            if format.has_color {
                assert_eq!(
                    Some(&ColumnData::U16(vec![raw_point.color.unwrap().red])),
                    batch.column(LasDimension::Red)
                );
                assert_eq!(
                    Some(&ColumnData::U16(vec![raw_point.color.unwrap().blue])),
                    batch.column(LasDimension::Blue)
                );
            }
            if format.has_nir {
                assert_eq!(
                    Some(&ColumnData::U16(vec![raw_point.nir.unwrap()])),
                    batch.column(LasDimension::Nir)
                );
            }
            assert_eq!(
                Some(&ColumnData::U8(vec![7, 9])),
                batch.column(LasDimension::ExtraBytes)
            );
        }
    }
}
