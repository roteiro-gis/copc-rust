use std::fs::File;
use std::io::{BufReader, BufWriter, Cursor, Seek, SeekFrom, Write};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use copc_core::{
    Bounds, CancelCheck, ColumnData, CopcInfo, Entry, Error, LasColumnBatch, LasDimension,
    LasPointRecord, NeverCancel, Result, StreamingLayout, VoxelKey, HIERARCHY_ENTRY_BYTES,
};
use las::{point::Format as LasFormat, raw, Color, Read as _};
use laz::{LasZipCompressor, LazVlrBuilder};
use tempfile::{NamedTempFile, TempPath};

use crate::spill::{SpillReader, SpillWriter};

const CANCEL_POLL_STRIDE: usize = 4_096;
const HIERARCHY_PAGE_MAX_ENTRIES: usize = 4_096;
const INDEX_RECORD_BYTES: u64 = 4;
/// Depth bounds for the LOD octree. The layered LAZ compressor buffers an
/// entire COPC chunk (one octree node) in memory before flushing, so a node
/// holding far more than `max_points_per_node` points costs proportional
/// memory. A too-shallow `max_depth` over a dense cluster stops subdivision
/// while a node is still huge — the multi-gigabyte failure mode on real clouds.
/// Clamping `max_depth` up to `MIN_LEAF_DEPTH` keeps nodes subdividing until
/// they fit in a chunk; realistic clouds reach that far shallower, so it only
/// affects pathologically dense input. `MAX_LEAF_DEPTH` keeps voxel keys in
/// range.
const MIN_LEAF_DEPTH: u32 = 21;
const MAX_LEAF_DEPTH: u32 = 30;
const LAS_14_SCAN_ANGLE_SCALE: f32 = 0.006;
const LAS_VLR_HEADER_BYTES: u32 = 54;
const LAS_EVLR_HEADER_BYTES: u64 = 60;
const LASZIP_VLR_USER_ID: &str = "laszip encoded";
const LASZIP_VLR_RECORD_ID: u16 = 22204;
const LASF_PROJECTION_USER_ID: &str = "LASF_Projection";
const WKT_CRS_RECORD_ID: u16 = 2112;
const GEOTIFF_GEO_KEY_DIRECTORY_RECORD_ID: u16 = 34735;
const GEOTIFF_DOUBLE_PARAMS_RECORD_ID: u16 = 34736;
const GEOTIFF_ASCII_PARAMS_RECORD_ID: u16 = 34737;
const WKT_GLOBAL_ENCODING_BIT: u16 = 16;

/// Normalized point fields consumed by the COPC writer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CopcPointFields {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: u16,
    pub return_number: u8,
    pub number_of_returns: u8,
    pub synthetic: u8,
    pub key_point: u8,
    pub withheld: u8,
    pub overlap: u8,
    pub scan_channel: u8,
    pub scan_direction_flag: u8,
    pub edge_of_flight_line: u8,
    pub classification: u8,
    pub user_data: u8,
    /// Scan angle in degrees; encoded as LAS 1.4 scaled scan angle on write.
    pub scan_angle: f32,
    pub point_source_id: u16,
    pub gps_time: f64,
    pub red: u16,
    pub green: u16,
    pub blue: u16,
}

/// Abstract point-data source for COPC emission.
pub trait CopcPointSource {
    fn len(&self) -> usize;
    fn xyz(&self, index: usize) -> (f64, f64, f64);
    fn fields(&self, index: usize) -> Result<CopcPointFields>;

    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// COPC writer source backed directly by a neutral LAS column batch.
pub struct ColumnBatchSource<'a> {
    batch: &'a LasColumnBatch,
    x: &'a [f64],
    y: &'a [f64],
    z: &'a [f64],
    intensity: Option<&'a [u16]>,
    return_number: Option<&'a [u8]>,
    number_of_returns: Option<&'a [u8]>,
    synthetic: Option<&'a [bool]>,
    key_point: Option<&'a [bool]>,
    withheld: Option<&'a [bool]>,
    overlap: Option<&'a [bool]>,
    scan_channel: Option<&'a [u8]>,
    scan_direction_flag: Option<&'a [bool]>,
    edge_of_flight_line: Option<&'a [bool]>,
    classification: Option<&'a [u8]>,
    user_data: Option<&'a [u8]>,
    scan_angle_rank: Option<&'a [i16]>,
    point_source_id: Option<&'a [u16]>,
    gps_time: Option<&'a [f64]>,
    red: Option<&'a [u16]>,
    green: Option<&'a [u16]>,
    blue: Option<&'a [u16]>,
}

impl<'a> ColumnBatchSource<'a> {
    pub fn new(batch: &'a LasColumnBatch) -> Result<Self> {
        batch.validate()?;
        validate_column_batch_writer_support(batch)?;

        let x = required_f64_column(batch, LasDimension::X)?;
        let y = required_f64_column(batch, LasDimension::Y)?;
        let z = required_f64_column(batch, LasDimension::Z)?;
        let red = optional_u16_column(batch, LasDimension::Red)?;
        let green = optional_u16_column(batch, LasDimension::Green)?;
        let blue = optional_u16_column(batch, LasDimension::Blue)?;
        validate_color_columns(red, green, blue)?;

        Ok(Self {
            batch,
            x,
            y,
            z,
            intensity: optional_u16_column(batch, LasDimension::Intensity)?,
            return_number: optional_u8_column(batch, LasDimension::ReturnNumber)?,
            number_of_returns: optional_u8_column(batch, LasDimension::NumberOfReturns)?,
            synthetic: optional_bool_column(batch, LasDimension::Synthetic)?,
            key_point: optional_bool_column(batch, LasDimension::KeyPoint)?,
            withheld: optional_bool_column(batch, LasDimension::Withheld)?,
            overlap: optional_bool_column(batch, LasDimension::Overlap)?,
            scan_channel: optional_u8_column(batch, LasDimension::ScanChannel)?,
            scan_direction_flag: optional_bool_column(batch, LasDimension::ScanDirectionFlag)?,
            edge_of_flight_line: optional_bool_column(batch, LasDimension::EdgeOfFlightLine)?,
            classification: optional_u8_column(batch, LasDimension::Classification)?,
            user_data: optional_u8_column(batch, LasDimension::UserData)?,
            scan_angle_rank: optional_i16_column(batch, LasDimension::ScanAngleRank)?,
            point_source_id: optional_u16_column(batch, LasDimension::PointSourceId)?,
            gps_time: optional_f64_column(batch, LasDimension::GpsTime)?,
            red,
            green,
            blue,
        })
    }

    pub fn batch(&self) -> &LasColumnBatch {
        self.batch
    }

    pub fn has_color(&self) -> bool {
        self.red.is_some() && self.green.is_some() && self.blue.is_some()
    }

    pub fn bounds(&self) -> Result<Bounds> {
        if self.is_empty() {
            return Err(Error::InvalidInput(
                "cannot compute bounds for empty column batch".into(),
            ));
        }
        let mut bounds = Bounds::point(self.x[0], self.y[0], self.z[0]);
        for index in 1..self.len() {
            bounds.extend(self.x[index], self.y[index], self.z[index]);
        }
        Ok(bounds)
    }
}

impl CopcPointSource for ColumnBatchSource<'_> {
    fn len(&self) -> usize {
        self.batch.len()
    }

    #[inline]
    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        (self.x[index], self.y[index], self.z[index])
    }

    fn fields(&self, index: usize) -> Result<CopcPointFields> {
        Ok(CopcPointFields {
            x: self.x[index],
            y: self.y[index],
            z: self.z[index],
            intensity: at_u16(self.intensity, index),
            return_number: at_u8(self.return_number, index),
            number_of_returns: at_u8(self.number_of_returns, index),
            synthetic: at_bool_u8(self.synthetic, index),
            key_point: at_bool_u8(self.key_point, index),
            withheld: at_bool_u8(self.withheld, index),
            overlap: at_bool_u8(self.overlap, index),
            scan_channel: at_u8(self.scan_channel, index),
            scan_direction_flag: at_bool_u8(self.scan_direction_flag, index),
            edge_of_flight_line: at_bool_u8(self.edge_of_flight_line, index),
            classification: at_u8(self.classification, index),
            user_data: at_u8(self.user_data, index),
            scan_angle: self
                .scan_angle_rank
                .map(|column| column[index] as f32 * 90.0 / 180.0)
                .unwrap_or(0.0),
            point_source_id: at_u16(self.point_source_id, index),
            gps_time: self.gps_time.map(|column| column[index]).unwrap_or(0.0),
            red: at_u16(self.red, index),
            green: at_u16(self.green, index),
            blue: at_u16(self.blue, index),
        })
    }
}

fn at_bool_u8(column: Option<&[bool]>, index: usize) -> u8 {
    column.map(|values| u8::from(values[index])).unwrap_or(0)
}

fn at_u8(column: Option<&[u8]>, index: usize) -> u8 {
    column.map(|values| values[index]).unwrap_or(0)
}

fn at_u16(column: Option<&[u16]>, index: usize) -> u16 {
    column.map(|values| values[index]).unwrap_or(0)
}

fn validate_column_batch_writer_support(batch: &LasColumnBatch) -> Result<()> {
    let unsupported: Vec<_> = batch
        .columns
        .iter()
        .filter_map(|(spec, _)| match spec.dimension {
            LasDimension::Nir => Some("NIR point data"),
            LasDimension::WaveformPacketDescriptorIndex
            | LasDimension::WaveformPacketByteOffset
            | LasDimension::WaveformPacketSize
            | LasDimension::WavePacketReturnPointWaveformLocation => Some("waveform point data"),
            LasDimension::ExtraBytes => Some("extra point bytes"),
            _ => None,
        })
        .collect();
    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "COPC writer cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

fn validate_color_columns(
    red: Option<&[u16]>,
    green: Option<&[u16]>,
    blue: Option<&[u16]>,
) -> Result<()> {
    let present =
        usize::from(red.is_some()) + usize::from(green.is_some()) + usize::from(blue.is_some());
    if present == 0 || present == 3 {
        Ok(())
    } else {
        Err(Error::InvalidInput(
            "Red, Green, and Blue columns must be supplied together".into(),
        ))
    }
}

fn required_f64_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<&[f64]> {
    match batch.column(dimension) {
        Some(ColumnData::F64(values)) => Ok(values),
        Some(other) => Err(unexpected_column_type(dimension, "F64", other)),
        None => Err(Error::InvalidInput(format!(
            "ColumnBatchSource requires {dimension:?} column"
        ))),
    }
}

fn optional_f64_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[f64]>> {
    match batch.column(dimension) {
        Some(ColumnData::F64(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "F64", other)),
        None => Ok(None),
    }
}

fn optional_i16_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[i16]>> {
    match batch.column(dimension) {
        Some(ColumnData::I16(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "I16", other)),
        None => Ok(None),
    }
}

fn optional_u16_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[u16]>> {
    match batch.column(dimension) {
        Some(ColumnData::U16(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "U16", other)),
        None => Ok(None),
    }
}

fn optional_u8_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[u8]>> {
    match batch.column(dimension) {
        Some(ColumnData::U8(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "U8", other)),
        None => Ok(None),
    }
}

fn optional_bool_column(
    batch: &LasColumnBatch,
    dimension: LasDimension,
) -> Result<Option<&[bool]>> {
    match batch.column(dimension) {
        Some(ColumnData::Bool(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "Bool", other)),
        None => Ok(None),
    }
}

fn unexpected_column_type(dimension: LasDimension, expected: &str, actual: &ColumnData) -> Error {
    Error::InvalidInput(format!(
        "{dimension:?} column must be {expected}, found {:?}",
        actual.scalar()
    ))
}

struct SpillSource<'a> {
    reader: &'a SpillReader,
}

impl CopcPointSource for SpillSource<'_> {
    fn len(&self) -> usize {
        self.reader.len()
    }

    #[inline]
    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        self.reader.xyz_at(index)
    }

    fn fields(&self, index: usize) -> Result<CopcPointFields> {
        let record = self.reader.record_at(index)?;
        Ok(CopcPointFields {
            x: record.x,
            y: record.y,
            z: record.z,
            intensity: record.intensity,
            return_number: record.return_number,
            number_of_returns: record.number_of_returns,
            synthetic: u8::from(record.synthetic),
            key_point: u8::from(record.key_point),
            withheld: u8::from(record.withheld),
            overlap: u8::from(record.overlap),
            scan_channel: record.scan_channel,
            scan_direction_flag: u8::from(record.scan_direction_flag),
            edge_of_flight_line: u8::from(record.edge_of_flight_line),
            classification: record.classification,
            user_data: record.user_data,
            scan_angle: record.scan_angle,
            point_source_id: record.point_source_id,
            gps_time: record.gps_time,
            red: record.red,
            green: record.green,
            blue: record.blue,
        })
    }
}

#[derive(Clone, Debug)]
struct OutputCrsRecord {
    vlr: las::Vlr,
    is_extended: bool,
}

#[derive(Clone, Debug)]
struct OutputLasMetadata {
    file_source_id: u16,
    global_encoding: u16,
    guid: [u8; 16],
    system_identifier: String,
    generating_software: String,
    creation_day_of_year: u16,
    creation_year: u16,
    scale: (f64, f64, f64),
    offset: Option<(f64, f64, f64)>,
    crs_records: Vec<OutputCrsRecord>,
}

impl Default for OutputLasMetadata {
    fn default() -> Self {
        Self {
            file_source_id: 0,
            global_encoding: 0,
            guid: [0; 16],
            system_identifier: "copc-rust".to_string(),
            generating_software: "copc-writer".to_string(),
            creation_day_of_year: 0,
            creation_year: 2026,
            scale: (0.001, 0.001, 0.001),
            offset: None,
            crs_records: Vec::new(),
        }
    }
}

impl OutputLasMetadata {
    fn from_las_header(header: &las::Header) -> Self {
        let mut global_encoding = u16::from(header.gps_time_type());
        if header.has_synthetic_return_numbers() {
            global_encoding |= 8;
        }
        let crs_records = extract_source_wkt_crs_records(header);
        if !crs_records.is_empty() {
            global_encoding |= WKT_GLOBAL_ENCODING_BIT;
        }
        let transforms = header.transforms();
        let (creation_day_of_year, creation_year) = header
            .date()
            .map(|date| {
                let year = date.format("%Y").to_string().parse().unwrap_or(0);
                let day = date.format("%j").to_string().parse().unwrap_or(0);
                (day, year)
            })
            .unwrap_or((0, 0));

        Self {
            file_source_id: header.file_source_id(),
            global_encoding,
            guid: *header.guid().as_bytes(),
            system_identifier: header.system_identifier().to_string(),
            generating_software: header.generating_software().to_string(),
            creation_day_of_year,
            creation_year,
            scale: (transforms.x.scale, transforms.y.scale, transforms.z.scale),
            offset: Some((
                transforms.x.offset,
                transforms.y.offset,
                transforms.z.offset,
            )),
            crs_records,
        }
    }

    fn regular_crs_vlrs(&self) -> impl Iterator<Item = &las::Vlr> {
        self.crs_records
            .iter()
            .filter(|record| !record.is_extended)
            .map(|record| &record.vlr)
    }

    fn extended_crs_evlrs(&self) -> impl Iterator<Item = &las::Vlr> {
        self.crs_records
            .iter()
            .filter(|record| record.is_extended)
            .map(|record| &record.vlr)
    }

    fn regular_crs_vlr_count(&self) -> usize {
        self.crs_records
            .iter()
            .filter(|record| !record.is_extended)
            .count()
    }

    fn extended_crs_evlr_count(&self) -> usize {
        self.crs_records
            .iter()
            .filter(|record| record.is_extended)
            .count()
    }

    fn regular_crs_vlr_bytes(&self) -> Result<u32> {
        self.regular_crs_vlrs().try_fold(0u32, |total, vlr| {
            let data_len = u16::try_from(vlr.data.len()).map_err(|_| {
                Error::InvalidInput(format!(
                    "regular WKT CRS VLR is too large: {} byte(s)",
                    vlr.data.len()
                ))
            })?;
            total
                .checked_add(LAS_VLR_HEADER_BYTES + u32::from(data_len))
                .ok_or_else(|| Error::InvalidInput("CRS VLR byte size overflow".into()))
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PointStats {
    gpstime_min: f64,
    gpstime_max: f64,
    extended_return_counts: [u64; 15],
}

impl PointStats {
    fn new() -> Self {
        Self {
            gpstime_min: f64::INFINITY,
            gpstime_max: f64::NEG_INFINITY,
            extended_return_counts: [0; 15],
        }
    }

    fn record(&mut self, index: usize, fields: &CopcPointFields) -> Result<()> {
        validate_finite_value(&format!("point {index} GPS time"), fields.gps_time)?;
        self.gpstime_min = self.gpstime_min.min(fields.gps_time);
        self.gpstime_max = self.gpstime_max.max(fields.gps_time);
        if (1..=15).contains(&fields.return_number) {
            self.extended_return_counts[usize::from(fields.return_number - 1)] += 1;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CopcWriterParams {
    pub max_points_per_node: u32,
    pub max_depth: u32,
}

impl Default for CopcWriterParams {
    fn default() -> Self {
        Self {
            max_points_per_node: 100_000,
            max_depth: 8,
        }
    }
}

pub fn write_source<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
) -> Result<()> {
    write_source_with_cancel(path, source, has_color, bounds, params, &NeverCancel)
}

pub fn write_source_with_cancel<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    cancel.check()?;
    if source.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty cloud to COPC".into(),
        ));
    }
    write_copc_inner(
        path,
        source,
        has_color,
        bounds,
        params,
        cancel,
        &OutputLasMetadata::default(),
    )
}

pub fn write_streaming_with_cancel<I>(
    path: &Path,
    layout: StreamingLayout,
    points: I,
    params: &CopcWriterParams,
    spill_dir: &Path,
    cancel: &dyn CancelCheck,
) -> Result<()>
where
    I: IntoIterator<Item = Result<LasPointRecord>>,
{
    cancel.check()?;
    validate_streaming_layout_supported(&layout)?;
    let mut spill = SpillWriter::create(spill_dir, layout)?;
    for (index, item) in points.into_iter().enumerate() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let record = item?;
        validate_record_coordinates(&record, index)?;
        spill.push(&record)?;
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(path, reader, params, cancel, &OutputLasMetadata::default())
}

pub fn convert_las_to_copc_streaming(
    las_path: &Path,
    copc_path: &Path,
    params: &CopcWriterParams,
    spill_dir: &Path,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    cancel.check()?;
    let mut reader = las::Reader::from_path(las_path).map_err(|e| Error::Las(e.to_string()))?;
    validate_las_conversion_supported(reader.header())?;
    let output_metadata = OutputLasMetadata::from_las_header(reader.header());
    let layout = StreamingLayout::from_las_format(*reader.header().point_format());
    let mut spill = SpillWriter::create(spill_dir, layout)?;
    for (index, result) in reader.points().enumerate() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let point = result.map_err(|e| Error::Las(e.to_string()))?;
        let record = LasPointRecord::from_las_point(&point);
        validate_record_coordinates(&record, index)?;
        spill.push(&record)?;
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(copc_path, reader, params, cancel, &output_metadata)
}

fn validate_streaming_layout_supported(layout: &StreamingLayout) -> Result<()> {
    let mut unsupported = Vec::new();
    if layout.has_nir {
        unsupported.push("NIR point data".to_string());
    }
    if layout.has_waveform {
        unsupported.push("waveform point data".to_string());
    }
    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "COPC writer cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

fn validate_las_conversion_supported(header: &las::Header) -> Result<()> {
    let mut unsupported = Vec::new();
    let format = header.point_format();
    if format.has_nir {
        unsupported.push("NIR point data".to_string());
    }
    if format.has_waveform {
        unsupported.push("waveform point data".to_string());
    }
    if format.extra_bytes > 0 {
        unsupported.push(format!("{} extra point byte(s)", format.extra_bytes));
    }
    let source_has_wkt_crs_record = has_wkt_crs_record(header);
    let mut geotiff_crs_record_count = 0usize;
    let mut unsupported_vlr_count = 0usize;
    for vlr in header.vlrs() {
        if is_laszip_vlr(vlr) || is_wkt_crs_vlr(vlr) {
            continue;
        }
        if is_geotiff_crs_vlr(vlr) {
            if !source_has_wkt_crs_record {
                geotiff_crs_record_count += 1;
            }
            continue;
        }
        unsupported_vlr_count += 1;
    }
    let mut unsupported_evlr_count = 0usize;
    for evlr in header.evlrs() {
        if is_wkt_crs_vlr(evlr) {
            continue;
        }
        if is_geotiff_crs_vlr(evlr) {
            if !source_has_wkt_crs_record {
                geotiff_crs_record_count += 1;
            }
            continue;
        }
        unsupported_evlr_count += 1;
    }
    if geotiff_crs_record_count > 0 {
        unsupported.push(format!(
            "{geotiff_crs_record_count} GeoTIFF CRS VLR/EVLR(s); GeoTIFF-to-WKT CRS conversion is not implemented in copc-writer"
        ));
    }
    if unsupported_vlr_count > 0 {
        unsupported.push(format!("{unsupported_vlr_count} VLR(s)"));
    }
    if unsupported_evlr_count > 0 {
        unsupported.push(format!("{unsupported_evlr_count} EVLR(s)"));
    }
    if !header.padding().is_empty() {
        unsupported.push(format!("{} header padding byte(s)", header.padding().len()));
    }
    if !header.vlr_padding().is_empty() {
        unsupported.push(format!(
            "{} VLR padding byte(s)",
            header.vlr_padding().len()
        ));
    }
    if !header.point_padding().is_empty() {
        unsupported.push(format!(
            "{} point padding byte(s)",
            header.point_padding().len()
        ));
    }

    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "LAS-to-COPC streaming conversion cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

fn is_laszip_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASZIP_VLR_USER_ID && vlr.record_id == LASZIP_VLR_RECORD_ID
}

fn is_wkt_crs_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASF_PROJECTION_USER_ID && vlr.record_id == WKT_CRS_RECORD_ID
}

fn is_geotiff_crs_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASF_PROJECTION_USER_ID
        && matches!(
            vlr.record_id,
            GEOTIFF_GEO_KEY_DIRECTORY_RECORD_ID
                | GEOTIFF_DOUBLE_PARAMS_RECORD_ID
                | GEOTIFF_ASCII_PARAMS_RECORD_ID
        )
}

fn has_wkt_crs_record(header: &las::Header) -> bool {
    header.vlrs().iter().any(is_wkt_crs_vlr) || header.evlrs().iter().any(is_wkt_crs_vlr)
}

fn extract_source_wkt_crs_records(header: &las::Header) -> Vec<OutputCrsRecord> {
    let mut records = Vec::new();
    for vlr in header.vlrs() {
        if is_wkt_crs_vlr(vlr) {
            records.push(OutputCrsRecord {
                vlr: vlr.clone(),
                is_extended: false,
            });
        }
    }
    for evlr in header.evlrs() {
        if is_wkt_crs_vlr(evlr) {
            records.push(OutputCrsRecord {
                vlr: evlr.clone(),
                is_extended: true,
            });
        }
    }
    records
}

fn validate_record_coordinates(record: &LasPointRecord, index: usize) -> Result<()> {
    validate_xyz_finite(index, record.x, record.y, record.z)
}

fn validate_coordinate_inputs<S: CopcPointSource>(
    source: &S,
    bounds: Bounds,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    cancel: &dyn CancelCheck,
) -> Result<PointStats> {
    validate_bounds(bounds)?;
    validate_transform(scale, offset)?;
    let mut stats = PointStats::new();
    for index in 0..source.len() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let (x, y, z) = source.xyz(index);
        validate_xyz_finite(index, x, y, z)?;
        quantize_xyz(index, x, y, z, scale, offset)?;

        let fields = source.fields(index)?;
        validate_xyz_finite(index, fields.x, fields.y, fields.z)?;
        quantize_xyz(index, fields.x, fields.y, fields.z, scale, offset)?;
        validate_scan_angle(index, fields.scan_angle)?;
        validate_point_flags(index, &fields)?;
        stats.record(index, &fields)?;
    }
    Ok(stats)
}

fn validate_point_flags(index: usize, fields: &CopcPointFields) -> Result<()> {
    validate_point_field_range(index, "return_number", fields.return_number, 0, 15)?;
    validate_point_field_range(index, "number_of_returns", fields.number_of_returns, 0, 15)?;
    validate_point_field_range(index, "synthetic", fields.synthetic, 0, 1)?;
    validate_point_field_range(index, "key_point", fields.key_point, 0, 1)?;
    validate_point_field_range(index, "withheld", fields.withheld, 0, 1)?;
    validate_point_field_range(index, "overlap", fields.overlap, 0, 1)?;
    validate_point_field_range(index, "scan_channel", fields.scan_channel, 0, 3)?;
    validate_point_field_range(
        index,
        "scan_direction_flag",
        fields.scan_direction_flag,
        0,
        1,
    )?;
    validate_point_field_range(
        index,
        "edge_of_flight_line",
        fields.edge_of_flight_line,
        0,
        1,
    )
}

fn validate_scan_angle(index: usize, value: f32) -> Result<()> {
    if !value.is_finite() {
        return Err(Error::InvalidInput(format!(
            "point {index} scan_angle must be finite, got {value}"
        )));
    }
    let scaled = value / LAS_14_SCAN_ANGLE_SCALE;
    if scaled < f32::from(i16::MIN) || scaled > f32::from(i16::MAX) {
        return Err(Error::InvalidInput(format!(
            "point {index} scan_angle {value} encodes to {scaled}, outside LAS i16 range"
        )));
    }
    Ok(())
}

fn validate_point_field_range(index: usize, name: &str, value: u8, min: u8, max: u8) -> Result<()> {
    if (min..=max).contains(&value) {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "point {index} {name} must be in {min}..={max}, got {value}"
        )))
    }
}

fn validate_bounds(bounds: Bounds) -> Result<()> {
    validate_finite_value("bounds min x", bounds.min.0)?;
    validate_finite_value("bounds min y", bounds.min.1)?;
    validate_finite_value("bounds min z", bounds.min.2)?;
    validate_finite_value("bounds max x", bounds.max.0)?;
    validate_finite_value("bounds max y", bounds.max.1)?;
    validate_finite_value("bounds max z", bounds.max.2)?;
    for (axis, min, max) in [
        ("x", bounds.min.0, bounds.max.0),
        ("y", bounds.min.1, bounds.max.1),
        ("z", bounds.min.2, bounds.max.2),
    ] {
        if min > max {
            return Err(Error::InvalidInput(format!(
                "bounds {axis} min {min} exceeds max {max}"
            )));
        }
        validate_finite_value(&format!("bounds {axis} span"), max - min)?;
    }
    Ok(())
}

fn validate_transform(scale: (f64, f64, f64), offset: (f64, f64, f64)) -> Result<()> {
    for (axis, value) in [("x", scale.0), ("y", scale.1), ("z", scale.2)] {
        if !value.is_finite() || value <= 0.0 {
            return Err(Error::InvalidInput(format!(
                "LAS {axis} scale must be finite and positive, got {value}"
            )));
        }
    }
    validate_finite_value("LAS x offset", offset.0)?;
    validate_finite_value("LAS y offset", offset.1)?;
    validate_finite_value("LAS z offset", offset.2)?;
    Ok(())
}

fn validate_xyz_finite(index: usize, x: f64, y: f64, z: f64) -> Result<()> {
    validate_point_axis_finite(index, "x", x)?;
    validate_point_axis_finite(index, "y", y)?;
    validate_point_axis_finite(index, "z", z)
}

fn validate_point_axis_finite(index: usize, axis: &str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate must be finite, got {value}"
        )))
    }
}

fn validate_finite_value(name: &str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "{name} must be finite, got {value}"
        )))
    }
}

fn quantize_xyz(
    index: usize,
    x: f64,
    y: f64,
    z: f64,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
) -> Result<(i32, i32, i32)> {
    Ok((
        quantize_axis(index, "x", x, scale.0, offset.0)?,
        quantize_axis(index, "y", y, scale.1, offset.1)?,
        quantize_axis(index, "z", z, scale.2, offset.2)?,
    ))
}

fn quantize_axis(index: usize, axis: &str, value: f64, scale: f64, offset: f64) -> Result<i32> {
    let scaled = ((value - offset) / scale).round();
    if !scaled.is_finite() {
        return Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate cannot be encoded with scale {scale} and offset {offset}"
        )));
    }
    if scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
        return Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate {value} encodes to {scaled}, outside LAS i32 range"
        )));
    }
    Ok(scaled as i32)
}

fn write_copc_from_spill(
    path: &Path,
    reader: SpillReader,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
    metadata: &OutputLasMetadata,
) -> Result<()> {
    cancel.check()?;
    validate_streaming_layout_supported(&reader.layout())?;
    if reader.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty cloud to COPC".into(),
        ));
    }
    let has_color = reader.layout().has_color;
    let bounds = reader.bounds();
    let source = SpillSource { reader: &reader };
    write_copc_inner(path, &source, has_color, bounds, params, cancel, metadata)
}

fn write_copc_inner<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
    metadata: &OutputLasMetadata,
) -> Result<()> {
    cancel.check()?;
    let point_format_id = if has_color { 7u8 } else { 6u8 };
    let point_format =
        LasFormat::new(point_format_id).map_err(|e| Error::Las(format!("point format: {e}")))?;
    let point_record_length = point_format.len();

    let (scale_x, scale_y, scale_z) = metadata.scale;
    let (offset_x, offset_y, offset_z) =
        metadata
            .offset
            .unwrap_or((bounds.min.0, bounds.min.1, bounds.min.2));
    let point_stats = validate_coordinate_inputs(
        source,
        bounds,
        (scale_x, scale_y, scale_z),
        (offset_x, offset_y, offset_z),
        cancel,
    )?;
    let (center, halfsize) = cube_from_bounds(&bounds);

    let lod_index = build_lod_index(source, center, halfsize, params, cancel)?;
    cancel.check()?;

    let var_vlr = LazVlrBuilder::default()
        .with_point_format(point_format_id, 0)
        .map_err(|e| Error::Las(format!("laz items: {e}")))?
        .with_variable_chunk_size()
        .build();
    let mut var_vlr_bytes = Vec::new();
    var_vlr
        .write_to(&mut var_vlr_bytes)
        .map_err(|e| Error::Las(format!("variable chunk LAZ VLR: {e}")))?;

    let copc_info_vlr_size = 160u16;
    let las_header_size = 375u32;
    let regular_crs_vlr_count = metadata.regular_crs_vlr_count();
    let regular_crs_vlr_bytes = metadata.regular_crs_vlr_bytes()?;
    let number_of_vlrs = u32::try_from(
        2usize
            .checked_add(regular_crs_vlr_count)
            .ok_or_else(|| Error::InvalidInput("VLR count overflow".into()))?,
    )
    .map_err(|_| Error::InvalidInput("VLR count overflow".into()))?;
    let number_of_evlrs = u32::try_from(
        1usize
            .checked_add(metadata.extended_crs_evlr_count())
            .ok_or_else(|| Error::InvalidInput("EVLR count overflow".into()))?,
    )
    .map_err(|_| Error::InvalidInput("EVLR count overflow".into()))?;
    let var_vlr_body_size = u16::try_from(var_vlr_bytes.len())
        .map_err(|_| Error::InvalidInput("LAZ VLR byte size exceeds LAS VLR limit".into()))?;
    let var_vlr_storage_bytes = LAS_VLR_HEADER_BYTES
        .checked_add(u32::from(var_vlr_body_size))
        .ok_or_else(|| Error::InvalidInput("LAZ VLR byte size overflow".into()))?;
    let total_vlr_bytes = (LAS_VLR_HEADER_BYTES + u32::from(copc_info_vlr_size))
        .checked_add(regular_crs_vlr_bytes)
        .and_then(|total| total.checked_add(var_vlr_storage_bytes))
        .ok_or_else(|| Error::InvalidInput("VLR byte size overflow".into()))?;
    let offset_to_point_data = las_header_size
        .checked_add(total_vlr_bytes)
        .ok_or_else(|| Error::InvalidInput("point data offset overflow".into()))?;

    let file = File::create(path).map_err(|e| Error::io("create COPC file", e))?;
    let mut writer = BufWriter::new(file);

    let header = LasHeader {
        point_data_format: point_format_id | 0x80,
        point_record_length,
        offset_to_point_data,
        number_of_vlrs,
        file_source_id: metadata.file_source_id,
        global_encoding: metadata.global_encoding,
        guid: metadata.guid,
        system_identifier: metadata.system_identifier.clone(),
        generating_software: metadata.generating_software.clone(),
        creation_day_of_year: metadata.creation_day_of_year,
        creation_year: metadata.creation_year,
        scale: (scale_x, scale_y, scale_z),
        offset: (offset_x, offset_y, offset_z),
        bounds,
        legacy_point_count: 0,
        total_point_count: source.len() as u64,
        offset_to_first_evlr: 0,
        number_of_evlrs,
        extended_return_counts: point_stats.extended_return_counts,
    };
    header.write(&mut writer)?;

    write_vlr_header(&mut writer, "copc", 1, copc_info_vlr_size, "COPC info")?;
    let copc_info_payload_start = writer
        .stream_position()
        .map_err(|e| Error::io("record COPC info payload offset", e))?;
    writer
        .write_all(&[0u8; 160])
        .map_err(|e| Error::io("write COPC info placeholder", e))?;

    for vlr in metadata.regular_crs_vlrs() {
        write_las_vlr(&mut writer, vlr)?;
    }

    write_vlr_header(
        &mut writer,
        LASZIP_VLR_USER_ID,
        LASZIP_VLR_RECORD_ID,
        var_vlr_body_size,
        "http://laszip.org",
    )?;
    writer
        .write_all(&var_vlr_bytes)
        .map_err(|e| Error::io("write LAZ VLR", e))?;

    let point_data_actual_start = writer
        .stream_position()
        .map_err(|e| Error::io("record point data offset", e))?;
    if point_data_actual_start as u32 != offset_to_point_data {
        return Err(Error::InvalidInput(format!(
            "VLR size accounting mismatch: at {point_data_actual_start}, expected {offset_to_point_data}"
        )));
    }

    let mut compressor = LasZipCompressor::new(&mut writer, var_vlr.clone())
        .map_err(|e| Error::Las(format!("compressor: {e}")))?;
    let mut hierarchy: Vec<Entry> = Vec::with_capacity(lod_index.nodes.len());
    let order_path: &Path = lod_index.order_path.as_ref();
    let mut index_reader =
        BufReader::new(File::open(order_path).map_err(|e| Error::io("open LOD order", e))?);
    let mut point_buf = vec![0u8; point_record_length as usize];
    let mut chunk_start_file_offset = compressor
        .get_mut()
        .stream_position()
        .map_err(|e| Error::io("record chunk start", e))?;
    chunk_start_file_offset += 8;

    for node in &lod_index.nodes {
        cancel.check()?;
        index_reader
            .seek(SeekFrom::Start(node.start))
            .map_err(|e| Error::io("seek LOD order", e))?;
        for point_index in 0..node.count {
            if point_index % CANCEL_POLL_STRIDE == 0 {
                cancel.check()?;
            }
            let source_index = index_reader
                .read_u32::<LittleEndian>()
                .map_err(|e| Error::io("read LOD order", e))?
                as usize;
            let fields = source.fields(source_index as usize)?;
            encode_point_record(
                &mut point_buf,
                &fields,
                (scale_x, scale_y, scale_z),
                (offset_x, offset_y, offset_z),
                source_index as usize,
                &point_format,
            )?;
            compressor
                .compress_one(&point_buf)
                .map_err(|e| Error::Las(format!("compress point: {e}")))?;
        }
        compressor
            .finish_current_chunk()
            .map_err(|e| Error::Las(format!("finish chunk: {e}")))?;
        let after = compressor
            .get_mut()
            .stream_position()
            .map_err(|e| Error::io("record chunk end", e))?;
        let byte_size = i32::try_from(after - chunk_start_file_offset)
            .map_err(|_| Error::InvalidInput("LAZ chunk exceeds COPC i32 byte size".into()))?;
        let point_count = i32::try_from(node.count)
            .map_err(|_| Error::InvalidInput("node point count exceeds COPC i32 range".into()))?;
        hierarchy.push(Entry {
            key: node.key,
            offset: chunk_start_file_offset,
            byte_size,
            point_count,
        });
        chunk_start_file_offset = after;
    }

    cancel.check()?;
    compressor
        .done()
        .map_err(|e| Error::Las(format!("finish compressor: {e}")))?;
    drop(compressor);

    let first_evlr_start = writer
        .stream_position()
        .map_err(|e| Error::io("record first EVLR start", e))?;
    for evlr in metadata.extended_crs_evlrs() {
        write_las_evlr(&mut writer, evlr)?;
    }
    let hierarchy_evlr_start = writer
        .stream_position()
        .map_err(|e| Error::io("record hierarchy EVLR start", e))?;
    let root_hier_offset = hierarchy_evlr_start
        .checked_add(LAS_EVLR_HEADER_BYTES)
        .ok_or_else(|| Error::InvalidInput("hierarchy EVLR offset overflow".into()))?;
    let mut hierarchy_pages = plan_hierarchy_pages(&hierarchy, VoxelKey::root())?;
    let hierarchy_end = assign_hierarchy_page_offsets(&mut hierarchy_pages, root_hier_offset)?;
    let hierarchy_body_size = hierarchy_end
        .checked_sub(root_hier_offset)
        .ok_or_else(|| Error::InvalidInput("hierarchy size overflow".into()))?;
    write_evlr_header(
        &mut writer,
        "copc",
        1000,
        hierarchy_body_size,
        "COPC hierarchy",
    )?;
    let actual_root_hier_offset = writer
        .stream_position()
        .map_err(|e| Error::io("record root hierarchy offset", e))?;
    if actual_root_hier_offset != root_hier_offset {
        return Err(Error::InvalidInput(format!(
            "hierarchy offset accounting mismatch: at {actual_root_hier_offset}, expected {root_hier_offset}"
        )));
    }
    write_hierarchy_page_tree(&mut writer, &hierarchy_pages)?;

    writer
        .seek(SeekFrom::Start(copc_info_payload_start))
        .map_err(|e| Error::io("seek COPC info payload", e))?;
    let info = CopcInfo {
        center,
        halfsize,
        spacing: halfsize / 128.0,
        root_hier_offset,
        root_hier_size: hierarchy_pages.byte_size,
        gpstime_min: point_stats.gpstime_min,
        gpstime_max: point_stats.gpstime_max,
    };
    writer
        .write_all(&info.write_le_bytes())
        .map_err(|e| Error::io("patch COPC info", e))?;

    writer
        .seek(SeekFrom::Start(235))
        .map_err(|e| Error::io("seek first EVLR offset", e))?;
    writer
        .write_u64::<LittleEndian>(first_evlr_start)
        .map_err(|e| Error::io("patch first EVLR offset", e))?;

    writer
        .flush()
        .map_err(|e| Error::io("flush COPC file", e))?;
    Ok(())
}

#[derive(Debug)]
struct HierarchyPagePlan {
    key: VoxelKey,
    items: Vec<HierarchyPageItem>,
    offset: u64,
    byte_size: u64,
}

#[derive(Debug)]
enum HierarchyPageItem {
    Point(Entry),
    Child(Box<HierarchyPagePlan>),
}

fn plan_hierarchy_pages(entries: &[Entry], key: VoxelKey) -> Result<HierarchyPagePlan> {
    if entries.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty hierarchy page".into(),
        ));
    }
    if entries.len() <= HIERARCHY_PAGE_MAX_ENTRIES {
        return Ok(HierarchyPagePlan {
            key,
            items: entries
                .iter()
                .copied()
                .map(HierarchyPageItem::Point)
                .collect(),
            offset: 0,
            byte_size: 0,
        });
    }

    let mut point_entry = None;
    let mut child_entries: [Vec<Entry>; 8] = std::array::from_fn(|_| Vec::new());
    for entry in entries.iter().copied() {
        if entry.key == key {
            point_entry = Some(entry);
            continue;
        }
        let mut matched = false;
        for (octant, child_entries) in child_entries.iter_mut().enumerate() {
            let child_key = key.child(octant as u8);
            if key_contains(child_key, entry.key) {
                child_entries.push(entry);
                matched = true;
                break;
            }
        }
        if !matched {
            return Err(Error::InvalidInput(format!(
                "hierarchy entry {:?} is not under page key {:?}",
                entry.key, key
            )));
        }
    }

    let mut items = Vec::new();
    if let Some(entry) = point_entry {
        items.push(HierarchyPageItem::Point(entry));
    }
    for (octant, child_entries) in child_entries.into_iter().enumerate() {
        if child_entries.is_empty() {
            continue;
        }
        items.push(HierarchyPageItem::Child(Box::new(plan_hierarchy_pages(
            &child_entries,
            key.child(octant as u8),
        )?)));
    }
    if items.len() > HIERARCHY_PAGE_MAX_ENTRIES {
        return Err(Error::InvalidInput(format!(
            "hierarchy page for {:?} has {} entries, max is {}",
            key,
            items.len(),
            HIERARCHY_PAGE_MAX_ENTRIES
        )));
    }
    Ok(HierarchyPagePlan {
        key,
        items,
        offset: 0,
        byte_size: 0,
    })
}

fn assign_hierarchy_page_offsets(page: &mut HierarchyPagePlan, offset: u64) -> Result<u64> {
    page.offset = offset;
    page.byte_size = hierarchy_page_byte_size(page.items.len())?;
    let mut next = offset
        .checked_add(page.byte_size)
        .ok_or_else(|| Error::InvalidInput("hierarchy page offset overflow".into()))?;
    for item in &mut page.items {
        if let HierarchyPageItem::Child(child) = item {
            next = assign_hierarchy_page_offsets(child, next)?;
        }
    }
    Ok(next)
}

fn hierarchy_page_byte_size(entry_count: usize) -> Result<u64> {
    let bytes = entry_count
        .checked_mul(HIERARCHY_ENTRY_BYTES)
        .ok_or_else(|| Error::InvalidInput("hierarchy page size overflow".into()))?;
    u64::try_from(bytes).map_err(|_| Error::InvalidInput("hierarchy page is too large".into()))
}

fn write_hierarchy_page_tree<W: Write + Seek>(
    writer: &mut W,
    page: &HierarchyPagePlan,
) -> Result<()> {
    let position = writer
        .stream_position()
        .map_err(|e| Error::io("record hierarchy page offset", e))?;
    if position != page.offset {
        return Err(Error::InvalidInput(format!(
            "hierarchy page offset mismatch: at {position}, expected {}",
            page.offset
        )));
    }
    let mut entry_buf = [0u8; HIERARCHY_ENTRY_BYTES];
    for item in &page.items {
        hierarchy_page_item_entry(item)?.write_le(&mut entry_buf)?;
        writer
            .write_all(&entry_buf)
            .map_err(|e| Error::io("write hierarchy entry", e))?;
    }
    for item in &page.items {
        if let HierarchyPageItem::Child(child) = item {
            write_hierarchy_page_tree(writer, child)?;
        }
    }
    Ok(())
}

fn hierarchy_page_item_entry(item: &HierarchyPageItem) -> Result<Entry> {
    match item {
        HierarchyPageItem::Point(entry) => Ok(*entry),
        HierarchyPageItem::Child(child) => Ok(Entry {
            key: child.key,
            offset: child.offset,
            byte_size: i32::try_from(child.byte_size).map_err(|_| {
                Error::InvalidInput("child hierarchy page exceeds COPC i32 byte size".into())
            })?,
            point_count: -1,
        }),
    }
}

fn key_contains(ancestor: VoxelKey, key: VoxelKey) -> bool {
    if key.level < ancestor.level {
        return false;
    }
    let shift = (key.level - ancestor.level) as u32;
    (key.x >> shift) == ancestor.x
        && (key.y >> shift) == ancestor.y
        && (key.z >> shift) == ancestor.z
}

struct LodIndex {
    nodes: Vec<LodNodeRange>,
    order_path: TempPath,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LodNodeRange {
    key: VoxelKey,
    start: u64,
    count: usize,
}

struct IndexRun {
    path: TempPath,
    start: u64,
    count: usize,
}

fn build_lod_index<S: CopcPointSource>(
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
    let max_points_per_node = params.max_points_per_node.max(1) as usize;
    let max_depth = params.max_depth.clamp(MIN_LEAF_DEPTH, MAX_LEAF_DEPTH);
    let root_run = write_root_index_run(total_points, cancel)?;
    let mut order_file = new_index_tempfile("order")?;
    let mut order_offset = 0;
    let mut nodes = Vec::new();
    {
        let mut order_writer = BufWriter::new(order_file.as_file_mut());
        let mut builder = LodIndexBuilder {
            source,
            max_points_per_node,
            max_depth,
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
    max_depth: u32,
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
        if run.count <= self.max_points_per_node || key.level as u32 >= self.max_depth {
            let start = *self.order_offset;
            append_index_run_to_order(&run, self.order_writer, self.order_offset, self.cancel)?;
            self.nodes.push(LodNodeRange {
                key,
                start,
                count: run.count,
            });
            return Ok(());
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
                key.child(octant as u8),
                child_run,
                bounds.octant(octant as u8),
            )?;
        }
        Ok(())
    }
}

fn write_root_index_run(total_points: u32, cancel: &dyn CancelCheck) -> Result<IndexRun> {
    let mut writer = BufWriter::new(new_index_tempfile("root")?);
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
    for read_index in 0..run.count {
        if read_index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let index = reader
            .read_u32::<LittleEndian>()
            .map_err(|e| Error::io("read LOD partition index", e))?;
        let (x, y, z) = source.xyz(index as usize);
        let octant = child_octant(bounds, x, y, z);
        if writers[octant].is_none() {
            writers[octant] = Some(BufWriter::new(new_index_tempfile("partition")?));
        }
        writers[octant]
            .as_mut()
            .expect("partition writer exists")
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
                .expect("partition reader exists")
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
    Ok(BufReader::new(file))
}

fn new_index_tempfile(label: &str) -> Result<NamedTempFile> {
    let prefix = format!(".copc-writer-{label}.");
    tempfile::Builder::new()
        .prefix(&prefix)
        .suffix(".idx")
        .tempfile()
        .map_err(|e| Error::io("create LOD index file", e))
}

fn child_octant(bounds: Bounds, x: f64, y: f64, z: f64) -> usize {
    let center = bounds.center();
    usize::from(x >= center.0)
        | (usize::from(y >= center.1) << 1)
        | (usize::from(z >= center.2) << 2)
}

fn cube_from_bounds(bounds: &Bounds) -> ((f64, f64, f64), f64) {
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

struct LasHeader {
    point_data_format: u8,
    point_record_length: u16,
    offset_to_point_data: u32,
    number_of_vlrs: u32,
    file_source_id: u16,
    global_encoding: u16,
    guid: [u8; 16],
    system_identifier: String,
    generating_software: String,
    creation_day_of_year: u16,
    creation_year: u16,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    bounds: Bounds,
    legacy_point_count: u32,
    total_point_count: u64,
    offset_to_first_evlr: u64,
    number_of_evlrs: u32,
    extended_return_counts: [u64; 15],
}

impl LasHeader {
    fn write<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer
            .write_all(b"LASF")
            .map_err(|e| Error::io("write LAS signature", e))?;
        writer
            .write_u16::<LittleEndian>(self.file_source_id)
            .map_err(|e| Error::io("write file source id", e))?;
        writer
            .write_u16::<LittleEndian>(self.global_encoding)
            .map_err(|e| Error::io("write global encoding", e))?;
        writer
            .write_all(&self.guid)
            .map_err(|e| Error::io("write GUID", e))?;
        writer
            .write_u8(1)
            .map_err(|e| Error::io("write version major", e))?;
        writer
            .write_u8(4)
            .map_err(|e| Error::io("write version minor", e))?;
        writer
            .write_all(&pad(self.system_identifier.as_bytes(), 32))
            .map_err(|e| Error::io("write system id", e))?;
        writer
            .write_all(&pad(self.generating_software.as_bytes(), 32))
            .map_err(|e| Error::io("write generating software", e))?;
        writer
            .write_u16::<LittleEndian>(self.creation_day_of_year)
            .map_err(|e| Error::io("write creation day", e))?;
        writer
            .write_u16::<LittleEndian>(self.creation_year)
            .map_err(|e| Error::io("write creation year", e))?;
        writer
            .write_u16::<LittleEndian>(375)
            .map_err(|e| Error::io("write header size", e))?;
        writer
            .write_u32::<LittleEndian>(self.offset_to_point_data)
            .map_err(|e| Error::io("write point data offset", e))?;
        writer
            .write_u32::<LittleEndian>(self.number_of_vlrs)
            .map_err(|e| Error::io("write VLR count", e))?;
        writer
            .write_u8(self.point_data_format)
            .map_err(|e| Error::io("write point format", e))?;
        writer
            .write_u16::<LittleEndian>(self.point_record_length)
            .map_err(|e| Error::io("write point record length", e))?;
        writer
            .write_u32::<LittleEndian>(self.legacy_point_count)
            .map_err(|e| Error::io("write legacy point count", e))?;
        for _ in 0..5 {
            writer
                .write_u32::<LittleEndian>(0)
                .map_err(|e| Error::io("write legacy returns", e))?;
        }
        writer
            .write_f64::<LittleEndian>(self.scale.0)
            .map_err(|e| Error::io("write x scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.scale.1)
            .map_err(|e| Error::io("write y scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.scale.2)
            .map_err(|e| Error::io("write z scale", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.0)
            .map_err(|e| Error::io("write x offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.1)
            .map_err(|e| Error::io("write y offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.offset.2)
            .map_err(|e| Error::io("write z offset", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.0)
            .map_err(|e| Error::io("write max x", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.0)
            .map_err(|e| Error::io("write min x", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.1)
            .map_err(|e| Error::io("write max y", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.1)
            .map_err(|e| Error::io("write min y", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.max.2)
            .map_err(|e| Error::io("write max z", e))?;
        writer
            .write_f64::<LittleEndian>(self.bounds.min.2)
            .map_err(|e| Error::io("write min z", e))?;
        writer
            .write_u64::<LittleEndian>(0)
            .map_err(|e| Error::io("write waveform packet start", e))?;
        writer
            .write_u64::<LittleEndian>(self.offset_to_first_evlr)
            .map_err(|e| Error::io("write first EVLR offset", e))?;
        writer
            .write_u32::<LittleEndian>(self.number_of_evlrs)
            .map_err(|e| Error::io("write EVLR count", e))?;
        writer
            .write_u64::<LittleEndian>(self.total_point_count)
            .map_err(|e| Error::io("write total point count", e))?;
        for count in self.extended_return_counts {
            writer
                .write_u64::<LittleEndian>(count)
                .map_err(|e| Error::io("write extended returns", e))?;
        }
        Ok(())
    }
}

fn pad(value: &[u8], len: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(len);
    let take = value.len().min(len);
    out.extend_from_slice(&value[..take]);
    out.resize(len, 0);
    out
}

fn write_vlr_header<W: Write>(
    writer: &mut W,
    user_id: &str,
    record_id: u16,
    body_size: u16,
    description: &str,
) -> Result<()> {
    writer
        .write_u16::<LittleEndian>(0)
        .map_err(|e| Error::io("write VLR reserved", e))?;
    writer
        .write_all(&pad(user_id.as_bytes(), 16))
        .map_err(|e| Error::io("write VLR user id", e))?;
    writer
        .write_u16::<LittleEndian>(record_id)
        .map_err(|e| Error::io("write VLR record id", e))?;
    writer
        .write_u16::<LittleEndian>(body_size)
        .map_err(|e| Error::io("write VLR body size", e))?;
    writer
        .write_all(&pad(description.as_bytes(), 32))
        .map_err(|e| Error::io("write VLR description", e))?;
    Ok(())
}

fn write_las_vlr<W: Write>(writer: &mut W, vlr: &las::Vlr) -> Result<()> {
    let body_size = u16::try_from(vlr.data.len()).map_err(|_| {
        Error::InvalidInput(format!(
            "regular VLR {}:{} is too large: {} byte(s)",
            vlr.user_id,
            vlr.record_id,
            vlr.data.len()
        ))
    })?;
    write_vlr_header(
        writer,
        &vlr.user_id,
        vlr.record_id,
        body_size,
        &vlr.description,
    )?;
    writer
        .write_all(&vlr.data)
        .map_err(|e| Error::io("write VLR body", e))?;
    Ok(())
}

fn write_evlr_header<W: Write>(
    writer: &mut W,
    user_id: &str,
    record_id: u16,
    body_size: u64,
    description: &str,
) -> Result<()> {
    writer
        .write_u16::<LittleEndian>(0)
        .map_err(|e| Error::io("write EVLR reserved", e))?;
    writer
        .write_all(&pad(user_id.as_bytes(), 16))
        .map_err(|e| Error::io("write EVLR user id", e))?;
    writer
        .write_u16::<LittleEndian>(record_id)
        .map_err(|e| Error::io("write EVLR record id", e))?;
    writer
        .write_u64::<LittleEndian>(body_size)
        .map_err(|e| Error::io("write EVLR body size", e))?;
    writer
        .write_all(&pad(description.as_bytes(), 32))
        .map_err(|e| Error::io("write EVLR description", e))?;
    Ok(())
}

fn write_las_evlr<W: Write>(writer: &mut W, vlr: &las::Vlr) -> Result<()> {
    let body_size = u64::try_from(vlr.data.len()).map_err(|_| {
        Error::InvalidInput(format!(
            "EVLR {}:{} is too large: {} byte(s)",
            vlr.user_id,
            vlr.record_id,
            vlr.data.len()
        ))
    })?;
    write_evlr_header(
        writer,
        &vlr.user_id,
        vlr.record_id,
        body_size,
        &vlr.description,
    )?;
    writer
        .write_all(&vlr.data)
        .map_err(|e| Error::io("write EVLR body", e))?;
    Ok(())
}

fn encode_point_record(
    buf: &mut [u8],
    fields: &CopcPointFields,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    point_index: usize,
    format: &LasFormat,
) -> Result<()> {
    let mut cursor = Cursor::new(buf);
    let (ix, iy, iz) = quantize_xyz(point_index, fields.x, fields.y, fields.z, scale, offset)?;
    let flags =
        fields.synthetic | (fields.key_point << 1) | (fields.withheld << 2) | (fields.overlap << 3);
    let point = raw::Point {
        x: ix,
        y: iy,
        z: iz,
        intensity: fields.intensity,
        flags: raw::point::Flags::ThreeByte(
            fields.return_number | (fields.number_of_returns << 4),
            flags
                | (fields.scan_channel << 4)
                | (fields.scan_direction_flag << 6)
                | (fields.edge_of_flight_line << 7),
            fields.classification,
        ),
        scan_angle: raw::point::ScanAngle::from(fields.scan_angle),
        user_data: fields.user_data,
        point_source_id: fields.point_source_id,
        gps_time: Some(fields.gps_time),
        color: format
            .has_color
            .then_some(Color::new(fields.red, fields.green, fields.blue)),
        waveform: None,
        nir: None,
        extra_bytes: Vec::new(),
    };
    point
        .write_to(&mut cursor, format)
        .map_err(|e| Error::Las(format!("write point record: {e}")))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    struct VecSource {
        points: Vec<CopcPointFields>,
    }

    impl CopcPointSource for VecSource {
        fn len(&self) -> usize {
            self.points.len()
        }

        fn xyz(&self, index: usize) -> (f64, f64, f64) {
            let point = self.points[index];
            (point.x, point.y, point.z)
        }

        fn fields(&self, index: usize) -> Result<CopcPointFields> {
            Ok(self.points[index])
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
            })
            .collect();
        let source = VecSource { points };
        let bounds = source_bounds(&source);
        let (center, halfsize) = cube_from_bounds(&bounds);
        let params = CopcWriterParams {
            max_points_per_node: 7,
            max_depth: 5,
        };

        let spooled = build_lod_index(&source, center, halfsize, &params, &NeverCancel).unwrap();
        let ranges = read_lod_index(&spooled).unwrap();

        let mut seen = vec![false; source.len()];
        let mut total = 0usize;
        for (key, indices) in ranges {
            if key.level < params.max_depth as i32 {
                assert!(indices.len() <= params.max_points_per_node as usize);
            }
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
        // A dense cluster inside large bounds: at a shallow `max_depth` the whole
        // cluster would collapse into one oversized leaf, forcing the layered LAZ
        // compressor to buffer that entire chunk in memory (the multi-GB failure
        // mode on real clouds). The writer must keep subdividing dense nodes past
        // `max_depth` so no COPC chunk exceeds `max_points_per_node`.
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
        // Deliberately shallow — the writer must override it for the dense cluster.
        let params = CopcWriterParams {
            max_points_per_node: max_points as u32,
            max_depth: 3,
        };

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
    fn hierarchy_plan_splits_large_root_page() {
        let mut entries = vec![Entry {
            key: VoxelKey::root(),
            offset: 1,
            byte_size: 1,
            point_count: 1,
        }];
        let mut offset = 2;
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    entries.push(Entry {
                        key: VoxelKey { level: 4, x, y, z },
                        offset,
                        byte_size: 1,
                        point_count: 1,
                    });
                    offset += 1;
                }
            }
        }
        entries.sort_by_key(|entry| entry.key);

        let mut plan = plan_hierarchy_pages(&entries, VoxelKey::root()).unwrap();
        let start = 1024;
        let end = assign_hierarchy_page_offsets(&mut plan, start).unwrap();

        assert!(plan.byte_size < hierarchy_page_byte_size(entries.len()).unwrap());
        assert!(plan
            .items
            .iter()
            .any(|item| matches!(item, HierarchyPageItem::Child(_))));

        let mut out = Cursor::new(vec![0; start as usize]);
        out.seek(SeekFrom::Start(start)).unwrap();
        write_hierarchy_page_tree(&mut out, &plan).unwrap();
        assert_eq!(end, out.get_ref().len() as u64);
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
