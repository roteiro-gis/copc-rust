use std::fs::File;
use std::io::{BufWriter, Cursor, Seek, SeekFrom, Write};
use std::path::Path;

use byteorder::{LittleEndian, WriteBytesExt};
use copc_core::{
    Bounds, CancelCheck, CopcInfo, Entry, Error, LasPointRecord, NeverCancel, Result,
    StreamingLayout, VoxelKey,
};
use las::{point::Format as LasFormat, raw, Color, Read as _};
use laz::{LasZipCompressor, LazVlrBuilder};

use crate::spill::{SpillReader, SpillWriter};

const CANCEL_POLL_STRIDE: usize = 4_096;
const LAS_14_WKT_CRS_GLOBAL_ENCODING_BIT: u16 = 16;
const LASZIP_VLR_USER_ID: &str = "laszip encoded";
const LASZIP_VLR_RECORD_ID: u16 = 22204;

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
    pub scan_angle_rank: i16,
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
            scan_angle_rank: record.scan_angle,
            point_source_id: record.point_source_id,
            gps_time: record.gps_time,
            red: record.red,
            green: record.green,
            blue: record.blue,
        })
    }
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
}

impl Default for OutputLasMetadata {
    fn default() -> Self {
        Self {
            file_source_id: 0,
            global_encoding: LAS_14_WKT_CRS_GLOBAL_ENCODING_BIT,
            guid: [0; 16],
            system_identifier: "copc-rust".to_string(),
            generating_software: "copc-writer".to_string(),
            creation_day_of_year: 0,
            creation_year: 2026,
            scale: (0.001, 0.001, 0.001),
            offset: None,
        }
    }
}

impl OutputLasMetadata {
    fn from_las_header(header: &las::Header) -> Self {
        let mut global_encoding =
            u16::from(header.gps_time_type()) | LAS_14_WKT_CRS_GLOBAL_ENCODING_BIT;
        if header.has_synthetic_return_numbers() {
            global_encoding |= 8;
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
        }
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
    let unsupported_vlr_count = header
        .vlrs()
        .iter()
        .filter(|vlr| !is_laszip_vlr(vlr))
        .count();
    if unsupported_vlr_count > 0 {
        unsupported.push(format!("{unsupported_vlr_count} VLR(s)"));
    }
    if !header.evlrs().is_empty() {
        unsupported.push(format!("{} EVLR(s)", header.evlrs().len()));
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
        stats.record(index, &fields)?;
    }
    Ok(stats)
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

    let nodes = build_lod_nodes(source, center, halfsize, params, cancel)?;
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
    let total_vlr_bytes =
        (54u32 + u32::from(copc_info_vlr_size)) + (54u32 + var_vlr_bytes.len() as u32);
    let offset_to_point_data = las_header_size + total_vlr_bytes;

    let file = File::create(path).map_err(|e| Error::io("create COPC file", e))?;
    let mut writer = BufWriter::new(file);

    let header = LasHeader {
        point_data_format: point_format_id | 0x80,
        point_record_length,
        offset_to_point_data,
        number_of_vlrs: 2,
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
        number_of_evlrs: 1,
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

    write_vlr_header(
        &mut writer,
        LASZIP_VLR_USER_ID,
        LASZIP_VLR_RECORD_ID,
        var_vlr_bytes.len() as u16,
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
    let mut hierarchy: Vec<Entry> = Vec::with_capacity(nodes.len());
    let mut point_buf = vec![0u8; point_record_length as usize];
    let mut chunk_start_file_offset = compressor
        .get_mut()
        .stream_position()
        .map_err(|e| Error::io("record chunk start", e))?;
    chunk_start_file_offset += 8;

    for (key, indices) in &nodes {
        cancel.check()?;
        for (point_index, &source_index) in indices.iter().enumerate() {
            if point_index % CANCEL_POLL_STRIDE == 0 {
                cancel.check()?;
            }
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
        hierarchy.push(Entry {
            key: *key,
            offset: chunk_start_file_offset,
            byte_size: (after - chunk_start_file_offset) as i32,
            point_count: indices.len() as i32,
        });
        chunk_start_file_offset = after;
    }

    cancel.check()?;
    compressor
        .done()
        .map_err(|e| Error::Las(format!("finish compressor: {e}")))?;
    drop(compressor);

    let evlr_start = writer
        .stream_position()
        .map_err(|e| Error::io("record EVLR start", e))?;
    let hierarchy_body_size = (hierarchy.len() * 32) as u64;
    write_evlr_header(
        &mut writer,
        "copc",
        1000,
        hierarchy_body_size,
        "COPC hierarchy",
    )?;
    let root_hier_offset = writer
        .stream_position()
        .map_err(|e| Error::io("record root hierarchy offset", e))?;
    let mut entry_buf = [0u8; 32];
    for entry in &hierarchy {
        entry.write_le(&mut entry_buf)?;
        writer
            .write_all(&entry_buf)
            .map_err(|e| Error::io("write hierarchy entry", e))?;
    }

    writer
        .seek(SeekFrom::Start(copc_info_payload_start))
        .map_err(|e| Error::io("seek COPC info payload", e))?;
    let info = CopcInfo {
        center,
        halfsize,
        spacing: halfsize / 128.0,
        root_hier_offset,
        root_hier_size: hierarchy_body_size,
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
        .write_u64::<LittleEndian>(evlr_start)
        .map_err(|e| Error::io("patch first EVLR offset", e))?;

    writer
        .flush()
        .map_err(|e| Error::io("flush COPC file", e))?;
    Ok(())
}

fn build_lod_nodes<S: CopcPointSource>(
    source: &S,
    center: (f64, f64, f64),
    halfsize: f64,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
) -> Result<Vec<(VoxelKey, Vec<u32>)>> {
    cancel.check()?;
    let total_points = u32::try_from(source.len()).map_err(|_| {
        Error::InvalidInput("COPC writer supports at most u32::MAX points per file".into())
    })?;
    let max_points_per_node = params.max_points_per_node.max(1) as usize;
    let max_depth = params.max_depth.min(30);
    let mut builder = LodNodeBuilder {
        source,
        max_points_per_node,
        max_depth,
        cancel,
        nodes: Vec::new(),
    };
    builder.assign(
        VoxelKey::root(),
        (0..total_points).collect(),
        Bounds::cube(center, halfsize),
    )?;
    let mut nodes = builder.nodes;
    nodes.sort_by_key(|(key, _)| *key);
    Ok(nodes)
}

struct LodNodeBuilder<'a, S: CopcPointSource> {
    source: &'a S,
    max_points_per_node: usize,
    max_depth: u32,
    cancel: &'a dyn CancelCheck,
    nodes: Vec<(VoxelKey, Vec<u32>)>,
}

impl<S: CopcPointSource> LodNodeBuilder<'_, S> {
    fn assign(&mut self, key: VoxelKey, indices: Vec<u32>, bounds: Bounds) -> Result<()> {
        self.cancel.check()?;
        if indices.is_empty() {
            return Ok(());
        }
        if indices.len() <= self.max_points_per_node || key.level as u32 >= self.max_depth {
            self.nodes.push((key, indices));
            return Ok(());
        }

        let mut children: [Vec<u32>; 8] = std::array::from_fn(|_| Vec::new());
        for (partition_index, index) in indices.into_iter().enumerate() {
            if partition_index % 16_384 == 0 {
                self.cancel.check()?;
            }
            let (px, py, pz) = self.source.xyz(index as usize);
            children[child_octant(bounds, px, py, pz)].push(index);
        }
        for child in &mut children {
            child.reverse();
        }

        let mut selected = Vec::with_capacity(self.max_points_per_node);
        while selected.len() < self.max_points_per_node {
            let mut progressed = false;
            for child in &mut children {
                if let Some(index) = child.pop() {
                    selected.push(index);
                    progressed = true;
                    if selected.len() == self.max_points_per_node {
                        break;
                    }
                }
            }
            if !progressed {
                break;
            }
        }
        self.nodes.push((key, selected));

        for (octant, child_indices) in children.into_iter().enumerate() {
            if child_indices.is_empty() {
                continue;
            }
            self.assign(
                key.child(octant as u8),
                child_indices,
                bounds.octant(octant as u8),
            )?;
        }
        Ok(())
    }
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
    let rn = fields.return_number & 0x0F;
    let nr = fields.number_of_returns & 0x0F;
    let flags = (fields.synthetic & 1)
        | ((fields.key_point & 1) << 1)
        | ((fields.withheld & 1) << 2)
        | ((fields.overlap & 1) << 3);
    let chan = fields.scan_channel & 0x03;
    let sd = fields.scan_direction_flag & 1;
    let eof = fields.edge_of_flight_line & 1;
    let point = raw::Point {
        x: ix,
        y: iy,
        z: iz,
        intensity: fields.intensity,
        flags: raw::point::Flags::ThreeByte(
            rn | (nr << 4),
            flags | (chan << 4) | (sd << 6) | (eof << 7),
            fields.classification,
        ),
        scan_angle: raw::point::ScanAngle::from(fields.scan_angle_rank as f32),
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
