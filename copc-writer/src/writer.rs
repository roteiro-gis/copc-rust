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
    write_copc_inner(path, source, has_color, bounds, params, cancel)
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
    let mut spill = SpillWriter::create(spill_dir, layout)?;
    for (index, item) in points.into_iter().enumerate() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        spill.push(&item?)?;
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(path, reader, params, cancel)
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
    let layout = StreamingLayout::from_las_format(*reader.header().point_format());
    let mut spill = SpillWriter::create(spill_dir, layout)?;
    for (index, result) in reader.points().enumerate() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let point = result.map_err(|e| Error::Las(e.to_string()))?;
        let record = LasPointRecord::from_las_point(&point);
        spill.push(&record)?;
    }
    cancel.check()?;
    let reader = spill.finalize()?;
    write_copc_from_spill(copc_path, reader, params, cancel)
}

fn write_copc_from_spill(
    path: &Path,
    reader: SpillReader,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    cancel.check()?;
    if reader.is_empty() {
        return Err(Error::InvalidInput(
            "cannot write empty cloud to COPC".into(),
        ));
    }
    let has_color = reader.layout().has_color;
    let bounds = reader.bounds();
    let source = SpillSource { reader: &reader };
    write_copc_inner(path, &source, has_color, bounds, params, cancel)
}

fn write_copc_inner<S: CopcPointSource>(
    path: &Path,
    source: &S,
    has_color: bool,
    bounds: Bounds,
    params: &CopcWriterParams,
    cancel: &dyn CancelCheck,
) -> Result<()> {
    cancel.check()?;
    let point_format_id = if has_color { 7u8 } else { 6u8 };
    let point_format =
        LasFormat::new(point_format_id).map_err(|e| Error::Las(format!("point format: {e}")))?;
    let point_record_length = point_format.len();

    let (center, halfsize) = cube_from_bounds(&bounds);
    let (scale_x, scale_y, scale_z) = (0.001, 0.001, 0.001);
    let (offset_x, offset_y, offset_z) = (bounds.min.0, bounds.min.1, bounds.min.2);

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
        scale: (scale_x, scale_y, scale_z),
        offset: (offset_x, offset_y, offset_z),
        bounds,
        legacy_point_count: 0,
        total_point_count: source.len() as u64,
        offset_to_first_evlr: 0,
        number_of_evlrs: 1,
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
        "laszip encoded",
        22204,
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
        gpstime_min: 0.0,
        gpstime_max: 0.0,
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
    let center = bounds.center();
    let dx = bounds.max.0 - bounds.min.0;
    let dy = bounds.max.1 - bounds.min.1;
    let dz = bounds.max.2 - bounds.min.2;
    let halfsize = (dx.max(dy).max(dz) * 0.5).max(1e-6);
    (center, halfsize)
}

struct LasHeader {
    point_data_format: u8,
    point_record_length: u16,
    offset_to_point_data: u32,
    number_of_vlrs: u32,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    bounds: Bounds,
    legacy_point_count: u32,
    total_point_count: u64,
    offset_to_first_evlr: u64,
    number_of_evlrs: u32,
}

impl LasHeader {
    fn write<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer
            .write_all(b"LASF")
            .map_err(|e| Error::io("write LAS signature", e))?;
        writer
            .write_u16::<LittleEndian>(0)
            .map_err(|e| Error::io("write file source id", e))?;
        writer
            .write_u16::<LittleEndian>(0)
            .map_err(|e| Error::io("write global encoding", e))?;
        writer
            .write_u32::<LittleEndian>(0)
            .map_err(|e| Error::io("write GUID1", e))?;
        writer
            .write_u16::<LittleEndian>(0)
            .map_err(|e| Error::io("write GUID2", e))?;
        writer
            .write_u16::<LittleEndian>(0)
            .map_err(|e| Error::io("write GUID3", e))?;
        writer
            .write_all(&[0u8; 8])
            .map_err(|e| Error::io("write GUID4", e))?;
        writer
            .write_u8(1)
            .map_err(|e| Error::io("write version major", e))?;
        writer
            .write_u8(4)
            .map_err(|e| Error::io("write version minor", e))?;
        writer
            .write_all(&pad(b"copc-rust", 32))
            .map_err(|e| Error::io("write system id", e))?;
        writer
            .write_all(&pad(b"copc-writer", 32))
            .map_err(|e| Error::io("write generating software", e))?;
        writer
            .write_u16::<LittleEndian>(0)
            .map_err(|e| Error::io("write creation day", e))?;
        writer
            .write_u16::<LittleEndian>(2026)
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
        for _ in 0..15 {
            writer
                .write_u64::<LittleEndian>(0)
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
    format: &LasFormat,
) -> Result<()> {
    let mut cursor = Cursor::new(buf);
    let ix = ((fields.x - offset.0) / scale.0).round() as i32;
    let iy = ((fields.y - offset.1) / scale.1).round() as i32;
    let iz = ((fields.z - offset.2) / scale.2).round() as i32;
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
