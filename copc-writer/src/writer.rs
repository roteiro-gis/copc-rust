//! COPC write orchestration: entry points, chunk emission, and file assembly.

use std::fs::File;
use std::io::{BufReader, BufWriter, Seek, SeekFrom, Write};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use copc_core::{
    Bounds, CancelCheck, CopcInfo, Entry, Error, LasPointRecord, NeverCancel, Result,
    StreamingLayout, VoxelKey, MAX_EVLR_COUNT, MAX_VLR_COUNT,
};
use las::point::Format as LasFormat;
use laz::{LasZipCompressor, LazVlrBuilder};
use tempfile::NamedTempFile;

use crate::hierarchy_pages::{
    assign_hierarchy_page_offsets, plan_hierarchy_pages, write_hierarchy_page_tree,
};
use crate::las_out::{
    regular_las_vlrs_bytes, write_evlr_header, write_las_evlr, write_las_vlr, write_vlr_header,
    LasHeader, LAS_EVLR_HEADER_BYTES, LAS_VLR_HEADER_BYTES,
};
use crate::lod::{build_lod_index, cube_from_bounds, INDEX_IO_BUFFER_BYTES};
use crate::metadata::{
    read_all_source_evlrs, CopcWriteMetadata, OutputLasMetadata, LASZIP_VLR_RECORD_ID,
    LASZIP_VLR_USER_ID,
};
use crate::source::{CopcPointFields, CopcPointSource, SpillSource};
use crate::spill::{SpillReader, SpillWriter};
use crate::validate::{
    quantize_xyz, scan_angle_to_las_scaled, validate_las_conversion_supported,
    validate_source_points, validate_streaming_layout_supported, validate_write_setup, PointStats,
};
use crate::CANCEL_POLL_STRIDE;

const LAS_INPUT_BUFFER_BYTES: usize = 1024 * 1024;
const COPC_OUTPUT_BUFFER_BYTES: usize = 1024 * 1024;
const LAS_POINT_BATCH_SIZE: u64 = 64 * 1024;

/// Tuning parameters for COPC writes.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct CopcWriterParams {
    /// Target maximum number of points per octree node (one LAZ chunk). The
    /// octree subdivides until nodes fit this budget, up to an internal depth
    /// cap that keeps voxel keys in range.
    pub max_points_per_node: u32,
}

impl CopcWriterParams {
    pub fn new(max_points_per_node: u32) -> Self {
        Self {
            max_points_per_node,
        }
    }
}

impl Default for CopcWriterParams {
    fn default() -> Self {
        Self::new(100_000)
    }
}

pub fn write_source<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    metadata: &CopcWriteMetadata,
) -> Result<()> {
    write_source_with_cancel(
        path,
        source,
        has_color,
        bounds,
        params,
        metadata,
        &NeverCancel,
    )
}

pub fn write_source_with_cancel<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    metadata: &CopcWriteMetadata,
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
        &metadata.to_output(),
        None,
    )
}

pub fn write_streaming_with_cancel<I>(
    path: &Path,
    layout: StreamingLayout,
    points: I,
    params: &CopcWriterParams,
    metadata: &CopcWriteMetadata,
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
        spill.push(&item?)?;
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(path, reader, params, cancel, &metadata.to_output())
}

pub fn convert_las_to_copc_streaming(
    las_path: &Path,
    copc_path: &Path,
    params: &CopcWriterParams,
    spill_dir: &Path,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    convert_las_to_copc_streaming_inner(las_path, copc_path, params, spill_dir, cancel, None)
}

/// Converts LAS/LAZ to COPC and emits `crs_wkt_override` as a WKT CRS VLR
/// when the source has GeoTIFF CRS records but no WKT CRS record.
pub fn convert_las_to_copc_streaming_with_crs_wkt_override(
    las_path: &Path,
    copc_path: &Path,
    params: &CopcWriterParams,
    spill_dir: &Path,
    cancel: &dyn CancelCheck,
    crs_wkt_override: Option<&str>,
) -> Result<()> {
    convert_las_to_copc_streaming_inner(
        las_path,
        copc_path,
        params,
        spill_dir,
        cancel,
        crs_wkt_override,
    )
}

fn convert_las_to_copc_streaming_inner(
    las_path: &Path,
    copc_path: &Path,
    params: &CopcWriterParams,
    spill_dir: &Path,
    cancel: &dyn CancelCheck,
    crs_wkt_override: Option<&str>,
) -> Result<()> {
    cancel.check()?;
    let las_file = File::open(las_path).map_err(|e| Error::io("open source LAS/LAZ", e))?;
    let mut reader = las::Reader::new(BufReader::with_capacity(LAS_INPUT_BUFFER_BYTES, las_file))
        .map_err(|e| Error::Las(e.to_string()))?;
    let source_evlrs = read_all_source_evlrs(las_path)?;
    validate_las_conversion_supported(reader.header(), &source_evlrs, crs_wkt_override)?;
    let output_metadata =
        OutputLasMetadata::from_las_header(reader.header(), &source_evlrs, crs_wkt_override);
    let layout = StreamingLayout::from_las_header(reader.header());
    let mut spill = SpillWriter::create(spill_dir, layout)?;
    let mut point_data = las::PointDataBuilder::new()
        .for_header(reader.header())
        .build();
    let mut index = 0usize;
    loop {
        let count = reader
            .fill_points(LAS_POINT_BATCH_SIZE, &mut point_data)
            .map_err(|e| Error::Las(e.to_string()))?;
        if count == 0 {
            break;
        }
        for result in point_data.points() {
            if index % CANCEL_POLL_STRIDE == 0 {
                cancel.check()?;
            }
            let point = result.map_err(|e| Error::Las(e.to_string()))?;
            spill.push(&LasPointRecord::from_las_point(&point))?;
            index = index
                .checked_add(1)
                .ok_or_else(|| Error::InvalidInput("source point count exceeds usize".into()))?;
        }
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(copc_path, reader, params, cancel, &output_metadata)
}

fn write_copc_from_spill(
    path: &Path,
    reader: SpillReader,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
    metadata: &OutputLasMetadata,
) -> Result<()> {
    cancel.check()?;
    if params.max_points_per_node == 0 {
        return Err(Error::InvalidInput(
            "max_points_per_node must be greater than zero".into(),
        ));
    }
    validate_streaming_layout_supported(reader.layout())?;
    if reader.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty cloud to COPC".into(),
        ));
    }
    let has_color = reader.layout().has_color;
    let bounds = reader.bounds();
    let stats = reader.stats();
    let source = SpillSource::new(&reader);
    write_copc_inner(
        path,
        &source,
        has_color,
        bounds,
        params,
        cancel,
        metadata,
        Some(stats),
    )
}

#[allow(clippy::too_many_arguments)]
fn write_copc_inner<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
    metadata: &OutputLasMetadata,
    intake_stats: Option<PointStats>,
) -> Result<()> {
    cancel.check()?;
    if params.max_points_per_node == 0 {
        return Err(Error::InvalidInput(
            "max_points_per_node must be greater than zero".into(),
        ));
    }
    let point_format_id = if has_color { 7u8 } else { 6u8 };
    let mut point_format =
        LasFormat::new(point_format_id).map_err(|e| Error::Las(format!("point format: {e}")))?;
    let extra_byte_count = source.extra_byte_count();
    let point_record_length = point_format
        .len()
        .checked_add(extra_byte_count)
        .ok_or_else(|| {
            Error::InvalidInput(format!(
                "point record length with {extra_byte_count} extra bytes exceeds LAS u16 range"
            ))
        })?;
    point_format.extra_bytes = extra_byte_count;

    let (scale_x, scale_y, scale_z) = metadata.scale;
    let (offset_x, offset_y, offset_z) =
        metadata
            .offset
            .unwrap_or((bounds.min.0, bounds.min.1, bounds.min.2));
    validate_write_setup(
        bounds,
        (scale_x, scale_y, scale_z),
        (offset_x, offset_y, offset_z),
    )?;
    // Spill-backed sources validate records and accumulate stats at intake;
    // other sources need the full validation pass here. Quantization-range
    // failures for intake-validated sources surface during encoding instead,
    // where the atomic output rename still prevents partial files.
    let point_stats = match intake_stats {
        Some(stats) => stats,
        None => validate_source_points(
            source,
            bounds,
            (scale_x, scale_y, scale_z),
            (offset_x, offset_y, offset_z),
            cancel,
        )?,
    };
    let (center, halfsize) = cube_from_bounds(&bounds);

    let lod_index = build_lod_index(source, center, halfsize, params, cancel)?;
    cancel.check()?;

    let var_vlr = LazVlrBuilder::default()
        .with_point_format(point_format_id, extra_byte_count)
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
    let extra_bytes_vlrs = source.extra_bytes_vlrs();
    let extra_bytes_vlr_bytes = regular_las_vlrs_bytes(extra_bytes_vlrs)?;
    let pass_through_vlr_bytes = regular_las_vlrs_bytes(&metadata.pass_through_vlrs)?;
    let number_of_vlrs = u32::try_from(
        2usize
            .checked_add(regular_crs_vlr_count)
            .and_then(|count| count.checked_add(extra_bytes_vlrs.len()))
            .and_then(|count| count.checked_add(metadata.pass_through_vlrs.len()))
            .ok_or_else(|| Error::InvalidInput("VLR count overflow".into()))?,
    )
    .map_err(|_| Error::InvalidInput("VLR count overflow".into()))?;
    if number_of_vlrs > MAX_VLR_COUNT {
        return Err(Error::InvalidInput(format!(
            "output VLR count {number_of_vlrs} exceeds max supported {MAX_VLR_COUNT}"
        )));
    }
    let number_of_evlrs = u32::try_from(
        1usize
            .checked_add(metadata.source_evlr_count_after_hierarchy())
            .ok_or_else(|| Error::InvalidInput("EVLR count overflow".into()))?,
    )
    .map_err(|_| Error::InvalidInput("EVLR count overflow".into()))?;
    if number_of_evlrs > MAX_EVLR_COUNT {
        return Err(Error::InvalidInput(format!(
            "output EVLR count {number_of_evlrs} exceeds max supported {MAX_EVLR_COUNT}"
        )));
    }
    let var_vlr_body_size = u16::try_from(var_vlr_bytes.len())
        .map_err(|_| Error::InvalidInput("LAZ VLR byte size exceeds LAS VLR limit".into()))?;
    let var_vlr_storage_bytes = LAS_VLR_HEADER_BYTES
        .checked_add(u32::from(var_vlr_body_size))
        .ok_or_else(|| Error::InvalidInput("LAZ VLR byte size overflow".into()))?;
    let total_vlr_bytes = LAS_VLR_HEADER_BYTES
        .checked_add(u32::from(copc_info_vlr_size))
        .and_then(|total| total.checked_add(var_vlr_storage_bytes))
        .and_then(|total| total.checked_add(regular_crs_vlr_bytes))
        .and_then(|total| total.checked_add(extra_bytes_vlr_bytes))
        .and_then(|total| total.checked_add(pass_through_vlr_bytes))
        .ok_or_else(|| Error::InvalidInput("VLR byte size overflow".into()))?;
    let offset_to_point_data = las_header_size
        .checked_add(total_vlr_bytes)
        .ok_or_else(|| Error::InvalidInput("point data offset overflow".into()))?;

    let pending = PendingOutput::create(path)?;
    let file = pending.reopen()?;
    let mut writer = BufWriter::with_capacity(COPC_OUTPUT_BUFFER_BYTES, file);

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

    for vlr in metadata.regular_crs_vlrs() {
        write_las_vlr(&mut writer, vlr)?;
    }
    for vlr in extra_bytes_vlrs {
        write_las_vlr(&mut writer, vlr)?;
    }
    for vlr in &metadata.pass_through_vlrs {
        write_las_vlr(&mut writer, vlr)?;
    }

    let point_data_actual_start = writer
        .stream_position()
        .map_err(|e| Error::io("record point data offset", e))?;
    if point_data_actual_start as u32 != offset_to_point_data {
        return Err(Error::InvalidInput(format!(
            "VLR size accounting mismatch: at {point_data_actual_start}, expected {offset_to_point_data}"
        )));
    }

    let hierarchy = compress_nodes(
        &mut writer,
        &var_vlr,
        &lod_index,
        source,
        (scale_x, scale_y, scale_z),
        (offset_x, offset_y, offset_z),
        usize::from(point_record_length),
        &point_format,
        cancel,
    )?;

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
    for evlr in metadata.source_evlrs_after_hierarchy() {
        write_las_evlr(&mut writer, evlr)?;
    }

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
    let info_bytes = info.write_le_bytes()?;
    writer
        .write_all(&info_bytes)
        .map_err(|e| Error::io("patch COPC info", e))?;

    writer
        .seek(SeekFrom::Start(235))
        .map_err(|e| Error::io("seek first EVLR offset", e))?;
    writer
        .write_u64::<LittleEndian>(hierarchy_evlr_start)
        .map_err(|e| Error::io("patch first EVLR offset", e))?;

    writer
        .flush()
        .map_err(|e| Error::io("flush COPC file", e))?;
    let file = writer
        .into_inner()
        .map_err(|e| Error::io("flush COPC file", e.into_error()))?;
    file.sync_all()
        .map_err(|e| Error::io("sync COPC file", e))?;
    drop(file);
    pending.commit()?;
    Ok(())
}

/// Guards an in-progress output file so failed writes never leave a partial
/// COPC file at the destination: the file is written to a same-directory temp
/// name and atomically renamed into place on success.
struct PendingOutput {
    file: Option<NamedTempFile>,
    final_path: std::path::PathBuf,
}

impl PendingOutput {
    fn create(path: &Path) -> Result<Self> {
        let file_name = path.file_name().ok_or_else(|| {
            Error::InvalidInput(format!("output path {} has no file name", path.display()))
        })?;
        let mut prefix = std::ffi::OsString::from(".");
        prefix.push(file_name);
        prefix.push(".");
        let parent = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        let file = tempfile::Builder::new()
            .prefix(&prefix)
            .suffix(".part")
            .tempfile_in(parent)
            .map_err(|e| Error::io("create temporary COPC file", e))?;
        Ok(Self {
            file: Some(file),
            final_path: path.to_path_buf(),
        })
    }

    fn reopen(&self) -> Result<File> {
        self.file
            .as_ref()
            .ok_or_else(|| Error::InvalidInput("temporary COPC file already committed".into()))?
            .reopen()
            .map_err(|e| Error::io("open temporary COPC file", e))
    }

    fn commit(mut self) -> Result<()> {
        let file = self
            .file
            .take()
            .ok_or_else(|| Error::InvalidInput("temporary COPC file already committed".into()))?;
        file.persist(&self.final_path)
            .map_err(|e| Error::io("persist COPC file", e.error))?;
        sync_parent_directory(&self.final_path)?;
        Ok(())
    }
}

#[cfg(unix)]
fn sync_parent_directory(path: &Path) -> Result<()> {
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    File::open(parent)
        .and_then(|directory| directory.sync_all())
        .map_err(|e| Error::io("sync COPC output directory", e))
}

#[cfg(not(unix))]
fn sync_parent_directory(_path: &Path) -> Result<()> {
    Ok(())
}

/// Reads one node's ordered source indexes from the LOD order file and
/// encodes its points into `raw` (`node.count * record_len` bytes).
#[allow(clippy::too_many_arguments)]
fn encode_node_points<S: CopcPointSource>(
    node: &crate::lod::LodNodeRange,
    index_reader: &mut BufReader<File>,
    source: &S,
    fields: &mut CopcPointFields,
    raw: &mut Vec<u8>,
    record_len: usize,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    point_format: &LasFormat,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    raw.clear();
    let raw_len = node
        .count
        .checked_mul(record_len)
        .ok_or_else(|| Error::InvalidInput("node point buffer size overflows usize".into()))?;
    raw.resize(raw_len, 0);
    index_reader
        .seek(SeekFrom::Start(node.start))
        .map_err(|e| Error::io("seek LOD order", e))?;
    for point_index in 0..node.count {
        if point_index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let source_index = index_reader
            .read_u32::<LittleEndian>()
            .map_err(|e| Error::io("read LOD order", e))? as usize;
        source.fields_into(source_index, fields)?;
        encode_point_record(
            &mut raw[point_index * record_len..(point_index + 1) * record_len],
            fields,
            scale,
            offset,
            source_index,
            point_format,
        )?;
    }
    Ok(())
}

fn hierarchy_entry(key: VoxelKey, offset: u64, byte_size: u64, count: usize) -> Result<Entry> {
    Ok(Entry {
        key,
        offset,
        byte_size: i32::try_from(byte_size)
            .map_err(|_| Error::InvalidInput("LAZ chunk exceeds COPC i32 byte size".into()))?,
        point_count: i32::try_from(count)
            .map_err(|_| Error::InvalidInput("node point count exceeds COPC i32 range".into()))?,
    })
}

/// Compress each LOD node into one COPC chunk, returning the hierarchy
/// entries. Sequential implementation: one `LasZipCompressor` streams every
/// chunk in order.
#[cfg(not(feature = "parallel"))]
#[allow(clippy::too_many_arguments)]
fn compress_nodes<W: Write + Seek + Send + Sync, S: CopcPointSource>(
    writer: &mut W,
    var_vlr: &laz::LazVlr,
    lod_index: &crate::lod::LodIndex,
    source: &S,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    record_len: usize,
    point_format: &LasFormat,
    cancel: &dyn CancelCheck,
) -> Result<Vec<Entry>> {
    let mut compressor = LasZipCompressor::new(&mut *writer, var_vlr.clone())
        .map_err(|e| Error::Las(format!("compressor: {e}")))?;
    let mut hierarchy = Vec::with_capacity(lod_index.nodes.len());
    let order_path: &Path = lod_index.order_path.as_ref();
    let mut index_reader = BufReader::with_capacity(
        INDEX_IO_BUFFER_BYTES,
        File::open(order_path).map_err(|e| Error::io("open LOD order", e))?,
    );
    let mut raw = Vec::new();
    let mut fields = CopcPointFields::default();
    let mut chunk_start_file_offset = compressor
        .get_mut()
        .stream_position()
        .map_err(|e| Error::io("record chunk start", e))?;
    chunk_start_file_offset = chunk_start_file_offset
        .checked_add(8)
        .ok_or_else(|| Error::InvalidInput("LAZ point-data offset overflows u64".into()))?;

    for node in &lod_index.nodes {
        cancel.check()?;
        encode_node_points(
            node,
            &mut index_reader,
            source,
            &mut fields,
            &mut raw,
            record_len,
            scale,
            offset,
            point_format,
            cancel,
        )?;
        compressor
            .compress_many(&raw)
            .map_err(|e| Error::Las(format!("compress chunk: {e}")))?;
        compressor
            .finish_current_chunk()
            .map_err(|e| Error::Las(format!("finish chunk: {e}")))?;
        let after = compressor
            .get_mut()
            .stream_position()
            .map_err(|e| Error::io("record chunk end", e))?;
        hierarchy.push(hierarchy_entry(
            node.key,
            chunk_start_file_offset,
            after.checked_sub(chunk_start_file_offset).ok_or_else(|| {
                Error::InvalidData("LAZ compressor moved before the chunk start".into())
            })?,
            node.count,
        )?);
        chunk_start_file_offset = after;
    }

    cancel.check()?;
    compressor
        .done()
        .map_err(|e| Error::Las(format!("finish compressor: {e}")))?;
    Ok(hierarchy)
}

/// Compress each LOD node into one COPC chunk, returning the hierarchy
/// entries. Parallel implementation: node point buffers are encoded
/// sequentially in bounded batches, compressed on rayon workers (each node is
/// one standalone LAZ chunk), then written in node order; the LAZ chunk table
/// and its offset are emitted to match the sequential layout.
///
/// Peak memory is roughly `2 * batch * max_points_per_node * record_len`
/// bytes for the raw and compressed batch buffers, with
/// `batch = 2 * rayon::current_num_threads()`.
#[cfg(feature = "parallel")]
#[allow(clippy::too_many_arguments)]
fn compress_nodes<W: Write + Seek + Send, S: CopcPointSource>(
    writer: &mut W,
    var_vlr: &laz::LazVlr,
    lod_index: &crate::lod::LodIndex,
    source: &S,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    record_len: usize,
    point_format: &LasFormat,
    cancel: &dyn CancelCheck,
) -> Result<Vec<Entry>> {
    use laz::laszip::{ChunkTable, ChunkTableEntry};
    use rayon::prelude::*;

    let table_offset_position = writer
        .stream_position()
        .map_err(|e| Error::io("record chunk table offset position", e))?;
    writer
        .write_i64::<LittleEndian>(-1)
        .map_err(|e| Error::io("write chunk table offset placeholder", e))?;

    let mut hierarchy = Vec::with_capacity(lod_index.nodes.len());
    let mut chunk_table = ChunkTable::with_capacity(lod_index.nodes.len());
    let order_path: &Path = lod_index.order_path.as_ref();
    let mut index_reader = BufReader::with_capacity(
        INDEX_IO_BUFFER_BYTES,
        File::open(order_path).map_err(|e| Error::io("open LOD order", e))?,
    );
    let mut fields = CopcPointFields::default();
    let mut chunk_start_file_offset = table_offset_position + 8;
    let batch_size = rayon::current_num_threads().max(1) * 2;

    for batch in lod_index.nodes.chunks(batch_size) {
        cancel.check()?;
        let mut raw_chunks = Vec::with_capacity(batch.len());
        for node in batch {
            let mut raw = Vec::new();
            encode_node_points(
                node,
                &mut index_reader,
                source,
                &mut fields,
                &mut raw,
                record_len,
                scale,
                offset,
                point_format,
                cancel,
            )?;
            raw_chunks.push(raw);
        }

        let compressed: Vec<Result<Vec<u8>>> = raw_chunks
            .par_iter()
            .map(|raw| compress_standalone_chunk(raw, var_vlr))
            .collect();

        for (node, chunk) in batch.iter().zip(compressed) {
            let chunk = chunk?;
            writer
                .write_all(&chunk)
                .map_err(|e| Error::io("write LAZ chunk", e))?;
            hierarchy.push(hierarchy_entry(
                node.key,
                chunk_start_file_offset,
                chunk.len() as u64,
                node.count,
            )?);
            chunk_table.push(ChunkTableEntry {
                point_count: node.count as u64,
                byte_count: chunk.len() as u64,
            });
            chunk_start_file_offset = chunk_start_file_offset
                .checked_add(chunk.len() as u64)
                .ok_or_else(|| Error::InvalidInput("LAZ point-data offset overflows u64".into()))?;
        }
    }

    cancel.check()?;
    let chunk_table_position = writer
        .stream_position()
        .map_err(|e| Error::io("record chunk table position", e))?;
    chunk_table
        .write_to(&mut *writer, var_vlr)
        .map_err(|e| Error::io("write chunk table", e))?;
    let end_position = writer
        .stream_position()
        .map_err(|e| Error::io("record chunk table end", e))?;
    writer
        .seek(SeekFrom::Start(table_offset_position))
        .map_err(|e| Error::io("seek chunk table offset", e))?;
    let chunk_table_position = i64::try_from(chunk_table_position)
        .map_err(|_| Error::InvalidInput("LAZ chunk table offset exceeds i64 range".into()))?;
    writer
        .write_i64::<LittleEndian>(chunk_table_position)
        .map_err(|e| Error::io("patch chunk table offset", e))?;
    writer
        .seek(SeekFrom::Start(end_position))
        .map_err(|e| Error::io("seek end of point data", e))?;
    Ok(hierarchy)
}

/// Compress one node's raw points as a standalone variable-size LAZ chunk and
/// return exactly the chunk bytes (no chunk-table offset, no chunk table).
#[cfg(feature = "parallel")]
fn compress_standalone_chunk(raw_points: &[u8], var_vlr: &laz::LazVlr) -> Result<Vec<u8>> {
    let mut cursor = std::io::Cursor::new(Vec::new());
    let mut compressor = LasZipCompressor::new(&mut cursor, var_vlr.clone())
        .map_err(|e| Error::Las(format!("chunk compressor: {e}")))?;
    compressor
        .compress_many(raw_points)
        .map_err(|e| Error::Las(format!("compress chunk: {e}")))?;
    compressor
        .done()
        .map_err(|e| Error::Las(format!("finish chunk: {e}")))?;
    drop(compressor);
    let bytes = cursor.into_inner();
    // A LAZ point-data stream starts with an i64 offset to the chunk table;
    // the single chunk's bytes sit between that offset field and the table.
    let table_position = i64::from_le_bytes(
        bytes[0..8]
            .try_into()
            .map_err(|_| Error::InvalidData("truncated LAZ chunk stream".into()))?,
    );
    let table_position = usize::try_from(table_position)
        .map_err(|_| Error::InvalidData("invalid LAZ chunk table offset".into()))?;
    if table_position < 8 || table_position > bytes.len() {
        return Err(Error::InvalidData(
            "LAZ chunk table offset out of range".into(),
        ));
    }
    Ok(bytes[8..table_position].to_vec())
}

/// Encode one point directly into the PDRF 6/7 record layout, avoiding the
/// per-point `las::raw::Point` construction (and its `extra_bytes` clone) in
/// the hot compression loop.
fn encode_point_record(
    buf: &mut [u8],
    fields: &CopcPointFields,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    point_index: usize,
    format: &LasFormat,
) -> Result<()> {
    debug_assert!(format.is_extended && !format.has_nir && !format.has_waveform);
    debug_assert_eq!(usize::from(format.len()), buf.len());
    let (ix, iy, iz) = quantize_xyz(point_index, fields.x, fields.y, fields.z, scale, offset)?;
    buf[0..4].copy_from_slice(&ix.to_le_bytes());
    buf[4..8].copy_from_slice(&iy.to_le_bytes());
    buf[8..12].copy_from_slice(&iz.to_le_bytes());
    buf[12..14].copy_from_slice(&fields.intensity.to_le_bytes());
    buf[14] = fields.return_number | (fields.number_of_returns << 4);
    buf[15] = fields.synthetic
        | (fields.key_point << 1)
        | (fields.withheld << 2)
        | (fields.overlap << 3)
        | (fields.scan_channel << 4)
        | (fields.scan_direction_flag << 6)
        | (fields.edge_of_flight_line << 7);
    buf[16] = fields.classification;
    buf[17] = fields.user_data;
    buf[18..20].copy_from_slice(&scan_angle_to_las_scaled(fields.scan_angle).to_le_bytes());
    buf[20..22].copy_from_slice(&fields.point_source_id.to_le_bytes());
    buf[22..30].copy_from_slice(&fields.gps_time.to_le_bytes());
    let mut cursor = 30;
    if format.has_color {
        buf[30..32].copy_from_slice(&fields.red.to_le_bytes());
        buf[32..34].copy_from_slice(&fields.green.to_le_bytes());
        buf[34..36].copy_from_slice(&fields.blue.to_le_bytes());
        cursor = 36;
    }
    if fields.extra_bytes.len() != buf.len() - cursor {
        return Err(Error::InvalidInput(format!(
            "point {point_index} has {} extra byte(s), expected {}",
            fields.extra_bytes.len(),
            buf.len() - cursor
        )));
    }
    buf[cursor..].copy_from_slice(&fields.extra_bytes);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use las::{raw, Color};

    /// The direct encoder must stay byte-identical to `las::raw::Point`
    /// serialization for the PDRF 6/7 layouts the writer emits.
    #[test]
    fn direct_point_encoding_matches_las_raw_point() {
        let fields = CopcPointFields {
            x: 12.345,
            y: -67.89,
            z: 101.5,
            intensity: 0xBEEF,
            return_number: 3,
            number_of_returns: 5,
            synthetic: 1,
            key_point: 0,
            withheld: 1,
            overlap: 0,
            scan_channel: 2,
            scan_direction_flag: 1,
            edge_of_flight_line: 0,
            classification: 6,
            user_data: 0x42,
            scan_angle: -30.25,
            point_source_id: 0xCAFE,
            gps_time: 1.234e9,
            red: 1_000,
            green: 2_000,
            blue: 3_000,
            extra_bytes: Vec::new(),
        };
        let scale = (0.001, 0.001, 0.001);
        let offset = (0.0, 0.0, 0.0);

        for (format_id, extra_bytes) in [(6u8, 0u16), (6, 3), (7, 0), (7, 5)] {
            let mut format = LasFormat::new(format_id).unwrap();
            format.extra_bytes = extra_bytes;
            let mut fields = fields.clone();
            fields.extra_bytes = (0..extra_bytes).map(|byte| byte as u8 ^ 0xA5).collect();

            let mut direct = vec![0u8; usize::from(format.len())];
            encode_point_record(&mut direct, &fields, scale, offset, 0, &format).unwrap();

            let (ix, iy, iz) =
                quantize_xyz(0, fields.x, fields.y, fields.z, scale, offset).unwrap();
            let class_flags = fields.synthetic
                | (fields.key_point << 1)
                | (fields.withheld << 2)
                | (fields.overlap << 3);
            let reference_point = raw::Point {
                x: ix,
                y: iy,
                z: iz,
                intensity: fields.intensity,
                flags: raw::point::Flags::ThreeByte(
                    fields.return_number | (fields.number_of_returns << 4),
                    class_flags
                        | (fields.scan_channel << 4)
                        | (fields.scan_direction_flag << 6)
                        | (fields.edge_of_flight_line << 7),
                    fields.classification,
                ),
                scan_angle: raw::point::ScanAngle::Scaled(scan_angle_to_las_scaled(
                    fields.scan_angle,
                )),
                user_data: fields.user_data,
                point_source_id: fields.point_source_id,
                gps_time: Some(fields.gps_time),
                color: format.has_color.then_some(Color::new(
                    fields.red,
                    fields.green,
                    fields.blue,
                )),
                waveform: None,
                nir: None,
                extra_bytes: fields.extra_bytes.clone(),
            };
            let mut reference = Vec::with_capacity(usize::from(format.len()));
            reference_point.write_to(&mut reference, &format).unwrap();

            assert_eq!(
                reference, direct,
                "format {format_id} with {extra_bytes} extra byte(s)"
            );
        }
    }
}
