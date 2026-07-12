//! Pure-Rust COPC reader.
//!
//! Parses LAS/COPC metadata and exposes chunked-LAZ point iteration over COPC
//! hierarchy entries.

#![forbid(unsafe_code)]

mod points;
mod range_read;
mod ranged;

use std::collections::{BTreeMap, HashSet};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt};
use copc_core::{
    CopcInfo, Entry, EntryAvailability, Error, HierarchyPage, Result, VoxelKey,
    HIERARCHY_ENTRY_BYTES, MAX_EVLR_COUNT, MAX_VLR_COUNT,
};
use las::{Transform, Vector};
use laz::LazVlr;

pub use copc_core::{
    ColumnData, ColumnSelection, ColumnSpec, ColumnView, LasColumnBatch, LasDimension, ScalarType,
};
pub use points::{BoundsSelection, CopcReader, LodSelection, PointIter, PointQuery};
#[cfg(feature = "http")]
pub use range_read::HttpRangeReader;
pub use range_read::RangeRead;
pub use ranged::CopcRangeReader;

const LAS_HEADER_SIZE_14: u16 = 375;
const VLR_HEADER_BYTES: u64 = 54;
const EVLR_HEADER_BYTES: u64 = 60;
const MAX_HIERARCHY_PAGE_BYTES: u64 = 64 * 1024 * 1024;
const MAX_HIERARCHY_TOTAL_BYTES: u64 = 256 * 1024 * 1024;

/// A parsed COPC file.
#[derive(Debug, Clone)]
pub struct CopcFile {
    header: LasHeader,
    copc_info: CopcInfo,
    laszip_vlr: LazVlr,
    root_hierarchy: HierarchyPage,
    hierarchy: BTreeMap<VoxelKey, Entry>,
}

/// Minimal LAS header fields needed by COPC callers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LasHeader {
    pub point_data_record_format: u8,
    pub point_data_record_length: u16,
    pub offset_to_point_data: u32,
    pub number_of_vlrs: u32,
    pub x_scale_factor: f64,
    pub y_scale_factor: f64,
    pub z_scale_factor: f64,
    pub x_offset: f64,
    pub y_offset: f64,
    pub z_offset: f64,
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
    pub offset_to_first_evlr: u64,
    pub number_of_evlrs: u32,
    pub number_of_points: u64,
}

#[derive(Debug, Clone)]
struct Vlr {
    user_id: String,
    record_id: u16,
    data: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
struct EvlrRef {
    user_id: [u8; 16],
    record_id: u16,
    data_offset: u64,
}

impl CopcFile {
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let mut file = File::open(path.as_ref()).map_err(|e| Error::io("open COPC file", e))?;
        Self::from_reader(&mut file)
    }

    pub fn from_reader<R: Read + Seek>(reader: &mut R) -> Result<Self> {
        let file_len = reader_len(reader)?;
        let header = read_las_header(reader, file_len)?;
        let vlrs = read_vlrs(
            reader,
            header.number_of_vlrs,
            file_len,
            u64::from(header.offset_to_point_data),
        )?;
        let (copc_info, laszip_vlr) = extract_required_vlrs(&vlrs)?;
        let evlrs = read_evlr_refs(reader, &header, file_len)?;
        let root_evlr = evlrs
            .iter()
            .find(|evlr| trim_nul(&evlr.user_id) == "copc" && evlr.record_id == 1000)
            .copied()
            .ok_or_else(|| Error::InvalidData("missing COPC hierarchy EVLR".into()))?;
        if copc_info.root_hier_offset != root_evlr.data_offset {
            return Err(Error::InvalidData(format!(
                "COPC root hierarchy offset {} does not match EVLR data offset {}",
                copc_info.root_hier_offset, root_evlr.data_offset
            )));
        }
        let mut hierarchy_limits = HierarchyReadLimits::default();
        let root_hierarchy = read_hierarchy_page_at(
            reader,
            copc_info.root_hier_offset,
            copc_info.root_hier_size,
            file_len,
            &mut hierarchy_limits,
        )?;
        let mut hierarchy = BTreeMap::new();
        let mut visited_pages = HashSet::new();
        visited_pages.insert((copc_info.root_hier_offset, copc_info.root_hier_size));
        insert_hierarchy_pages(
            reader,
            &root_hierarchy,
            &mut hierarchy,
            &mut visited_pages,
            file_len,
            &mut hierarchy_limits,
        )?;
        validate_hierarchy_point_total(&hierarchy, header.number_of_points)?;
        Ok(Self {
            header,
            copc_info,
            laszip_vlr,
            root_hierarchy,
            hierarchy,
        })
    }

    pub fn header(&self) -> &LasHeader {
        &self.header
    }

    pub fn copc_info(&self) -> &CopcInfo {
        &self.copc_info
    }

    pub fn root_hierarchy(&self) -> &HierarchyPage {
        &self.root_hierarchy
    }

    /// Return all parsed hierarchy entries, including recursively loaded child pages.
    pub fn hierarchy_walk(&self) -> Vec<Entry> {
        self.hierarchy.values().copied().collect()
    }

    /// Return the full hierarchy index keyed by COPC voxel key.
    pub fn hierarchy(&self) -> &BTreeMap<VoxelKey, Entry> {
        &self.hierarchy
    }

    pub fn hierarchy_entries(&self) -> impl Iterator<Item = &Entry> {
        self.hierarchy.values()
    }

    pub(crate) fn laszip_vlr(&self) -> &LazVlr {
        &self.laszip_vlr
    }

    pub(crate) fn point_format(&self) -> Result<las::point::Format> {
        point_format_for(&self.header)
    }

    pub(crate) fn transforms(&self) -> Vector<Transform> {
        transforms_for(&self.header)
    }
}

pub(crate) fn point_format_for(header: &LasHeader) -> Result<las::point::Format> {
    let format_id = header.point_data_record_format & 0x7F;
    let mut format = las::point::Format::new(format_id).map_err(|e| Error::Las(e.to_string()))?;
    let base_len = format.len();
    if header.point_data_record_length < base_len {
        return Err(Error::InvalidData(format!(
            "point record length {} is smaller than point format {} base length {}",
            header.point_data_record_length, format_id, base_len
        )));
    }
    format.extra_bytes = header.point_data_record_length - base_len;
    Ok(format)
}

pub(crate) fn transforms_for(header: &LasHeader) -> Vector<Transform> {
    Vector {
        x: Transform {
            scale: header.x_scale_factor,
            offset: header.x_offset,
        },
        y: Transform {
            scale: header.y_scale_factor,
            offset: header.y_offset,
        },
        z: Transform {
            scale: header.z_scale_factor,
            offset: header.z_offset,
        },
    }
}

impl LasHeader {
    pub fn number_of_points(&self) -> u64 {
        self.number_of_points
    }
}

#[derive(Debug, Default)]
pub(crate) struct HierarchyReadLimits {
    total_bytes: u64,
}

impl HierarchyReadLimits {
    pub(crate) fn add_page(&mut self, byte_size: u64) -> Result<()> {
        if byte_size > MAX_HIERARCHY_PAGE_BYTES {
            return Err(Error::InvalidData(format!(
                "hierarchy page is {byte_size} bytes, max supported is {MAX_HIERARCHY_PAGE_BYTES}"
            )));
        }
        self.total_bytes = self
            .total_bytes
            .checked_add(byte_size)
            .ok_or_else(|| Error::InvalidData("hierarchy byte total overflow".into()))?;
        if self.total_bytes > MAX_HIERARCHY_TOTAL_BYTES {
            return Err(Error::InvalidData(format!(
                "hierarchy pages total {} bytes, max supported is {}",
                self.total_bytes, MAX_HIERARCHY_TOTAL_BYTES
            )));
        }
        Ok(())
    }
}

/// Extracts the COPC info and LASzip VLRs every COPC file must carry.
pub(crate) fn extract_required_vlrs(vlrs: &[Vlr]) -> Result<(CopcInfo, LazVlr)> {
    let copc_info_vlr = vlrs
        .iter()
        .find(|vlr| vlr.user_id == "copc" && vlr.record_id == 1)
        .ok_or_else(|| Error::InvalidData("missing COPC info VLR".into()))?;
    let copc_info = CopcInfo::from_le_bytes(&copc_info_vlr.data)?;
    let laszip_vlr = vlrs
        .iter()
        .find(|vlr| vlr.user_id == "laszip encoded" && vlr.record_id == 22204)
        .map(|vlr| LazVlr::read_from(vlr.data.as_slice()).map_err(|e| Error::Las(e.to_string())))
        .transpose()?
        .ok_or_else(|| Error::InvalidData("missing LASzip VLR".into()))?;
    Ok((copc_info, laszip_vlr))
}

fn reader_len<R: Seek>(reader: &mut R) -> Result<u64> {
    let current = reader
        .stream_position()
        .map_err(|e| Error::io("record reader position", e))?;
    let len = reader
        .seek(SeekFrom::End(0))
        .map_err(|e| Error::io("seek end of COPC file", e))?;
    reader
        .seek(SeekFrom::Start(current))
        .map_err(|e| Error::io("restore reader position", e))?;
    Ok(len)
}

fn checked_range_end(offset: u64, byte_size: u64, label: &str) -> Result<u64> {
    offset
        .checked_add(byte_size)
        .ok_or_else(|| Error::InvalidData(format!("{label} offset/size overflow")))
}

fn validate_range_in_file(offset: u64, byte_size: u64, file_len: u64, label: &str) -> Result<u64> {
    let end = checked_range_end(offset, byte_size, label)?;
    if end > file_len {
        return Err(Error::InvalidData(format!(
            "{label} range {offset}..{end} exceeds file length {file_len}"
        )));
    }
    Ok(end)
}

fn read_hierarchy_page_at<R: Read + Seek>(
    reader: &mut R,
    offset: u64,
    byte_size: u64,
    file_len: u64,
    limits: &mut HierarchyReadLimits,
) -> Result<HierarchyPage> {
    if byte_size == 0 {
        return Err(Error::InvalidData("hierarchy page is empty".into()));
    }
    if byte_size % HIERARCHY_ENTRY_BYTES as u64 != 0 {
        return Err(Error::InvalidData(format!(
            "hierarchy page is {byte_size} bytes, not a multiple of {HIERARCHY_ENTRY_BYTES}"
        )));
    }
    limits.add_page(byte_size)?;
    validate_range_in_file(offset, byte_size, file_len, "hierarchy page")?;
    let hierarchy_len = usize::try_from(byte_size)
        .map_err(|_| Error::InvalidData("hierarchy page is too large".into()))?;
    let mut hierarchy_bytes = vec![0u8; hierarchy_len];
    reader
        .seek(SeekFrom::Start(offset))
        .map_err(|e| Error::io("seek hierarchy page", e))?;
    reader
        .read_exact(&mut hierarchy_bytes)
        .map_err(|e| Error::io("read hierarchy page", e))?;
    HierarchyPage::from_le_bytes(&hierarchy_bytes)
}

/// Loads the full hierarchy tree with an explicit worklist rather than
/// recursion, so a crafted file with a long chain of child pages cannot
/// overflow the stack.
fn insert_hierarchy_pages<R: Read + Seek>(
    reader: &mut R,
    root_page: &HierarchyPage,
    hierarchy: &mut BTreeMap<VoxelKey, Entry>,
    visited_pages: &mut HashSet<(u64, u64)>,
    file_len: u64,
    limits: &mut HierarchyReadLimits,
) -> Result<()> {
    let mut pending = Vec::new();
    queue_hierarchy_page(root_page, hierarchy, visited_pages, &mut pending, file_len)?;
    while let Some((offset, byte_size)) = pending.pop() {
        let page = read_hierarchy_page_at(reader, offset, byte_size, file_len, limits)?;
        queue_hierarchy_page(&page, hierarchy, visited_pages, &mut pending, file_len)?;
    }
    Ok(())
}

fn queue_hierarchy_page(
    page: &HierarchyPage,
    hierarchy: &mut BTreeMap<VoxelKey, Entry>,
    visited_pages: &mut HashSet<(u64, u64)>,
    pending: &mut Vec<(u64, u64)>,
    file_len: u64,
) -> Result<()> {
    for entry in page.entries().iter().copied() {
        validate_hierarchy_entry(entry, file_len)?;
        hierarchy.insert(entry.key, entry);
    }
    for entry in page.entries().iter().copied().filter(|e| e.is_child_page()) {
        let byte_size = u64::try_from(entry.byte_size).expect("validated child page byte size");
        if visited_pages.insert((entry.offset, byte_size)) {
            pending.push((entry.offset, byte_size));
        }
    }
    Ok(())
}

/// Rejects hierarchies whose claimed point totals exceed the LAS header point
/// count, so downstream pre-allocations sized from chunk point counts stay
/// bounded by a value that is at least self-consistent.
fn validate_hierarchy_point_total(
    hierarchy: &BTreeMap<VoxelKey, Entry>,
    number_of_points: u64,
) -> Result<()> {
    let mut total: u64 = 0;
    for entry in hierarchy.values() {
        if let EntryAvailability::PointData { point_count } = entry.availability()? {
            total = total
                .checked_add(u64::from(point_count))
                .ok_or_else(|| Error::InvalidData("hierarchy point total overflows u64".into()))?;
        }
    }
    if total > number_of_points {
        return Err(Error::InvalidData(format!(
            "hierarchy point total {total} exceeds LAS header point count {number_of_points}"
        )));
    }
    Ok(())
}

fn validate_hierarchy_entry(entry: Entry, file_len: u64) -> Result<()> {
    match entry.availability()? {
        EntryAvailability::Empty => Ok(()),
        EntryAvailability::PointData { .. } => {
            if entry.byte_size <= 0 {
                return Err(Error::InvalidData(format!(
                    "point data entry {:?} has invalid byte size {}",
                    entry.key, entry.byte_size
                )));
            }
            let byte_size = u64::try_from(entry.byte_size).map_err(|_| {
                Error::InvalidData(format!(
                    "point data entry {:?} has negative byte size {}",
                    entry.key, entry.byte_size
                ))
            })?;
            validate_range_in_file(entry.offset, byte_size, file_len, "point data entry")?;
            Ok(())
        }
        EntryAvailability::ChildPage => {
            if entry.byte_size <= 0 {
                return Err(Error::InvalidData(format!(
                    "child hierarchy page {:?} has invalid byte size {}",
                    entry.key, entry.byte_size
                )));
            }
            let byte_size = u64::try_from(entry.byte_size).map_err(|_| {
                Error::InvalidData(format!(
                    "child hierarchy page {:?} has negative byte size {}",
                    entry.key, entry.byte_size
                ))
            })?;
            validate_range_in_file(entry.offset, byte_size, file_len, "child hierarchy page")?;
            Ok(())
        }
    }
}

fn read_las_header<R: Read + Seek>(reader: &mut R, file_len: u64) -> Result<LasHeader> {
    if file_len < u64::from(LAS_HEADER_SIZE_14) {
        return Err(Error::InvalidData(format!(
            "file is {file_len} bytes; COPC requires at least {LAS_HEADER_SIZE_14}"
        )));
    }
    reader
        .seek(SeekFrom::Start(0))
        .map_err(|e| Error::io("seek LAS header", e))?;
    let mut signature = [0u8; 4];
    reader
        .read_exact(&mut signature)
        .map_err(|e| Error::io("read LAS signature", e))?;
    if &signature != b"LASF" {
        return Err(Error::InvalidData("missing LASF signature".into()));
    }
    reader
        .seek(SeekFrom::Start(94))
        .map_err(|e| Error::io("seek LAS header size", e))?;
    let header_size = reader
        .read_u16::<LittleEndian>()
        .map_err(|e| Error::io("read LAS header size", e))?;
    if header_size < LAS_HEADER_SIZE_14 {
        return Err(Error::Unsupported(format!(
            "LAS header is {header_size} bytes; COPC requires LAS 1.4"
        )));
    }
    if u64::from(header_size) > file_len {
        return Err(Error::InvalidData(format!(
            "LAS header size {header_size} exceeds file length {file_len}"
        )));
    }
    let offset_to_point_data = reader
        .read_u32::<LittleEndian>()
        .map_err(|e| Error::io("read point data offset", e))?;
    if u64::from(offset_to_point_data) < u64::from(header_size) {
        return Err(Error::InvalidData(format!(
            "point data offset {offset_to_point_data} is before LAS header size {header_size}"
        )));
    }
    if u64::from(offset_to_point_data) > file_len {
        return Err(Error::InvalidData(format!(
            "point data offset {offset_to_point_data} exceeds file length {file_len}"
        )));
    }
    let number_of_vlrs = reader
        .read_u32::<LittleEndian>()
        .map_err(|e| Error::io("read VLR count", e))?;
    if number_of_vlrs > MAX_VLR_COUNT {
        return Err(Error::InvalidData(format!(
            "VLR count {number_of_vlrs} exceeds max supported {MAX_VLR_COUNT}"
        )));
    }
    let point_data_record_format = reader
        .read_u8()
        .map_err(|e| Error::io("read point record format", e))?;
    let point_data_record_length = reader
        .read_u16::<LittleEndian>()
        .map_err(|e| Error::io("read point record length", e))?;
    reader
        .seek(SeekFrom::Start(131))
        .map_err(|e| Error::io("seek LAS transforms", e))?;
    let x_scale_factor = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read x scale factor", e))?;
    let y_scale_factor = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read y scale factor", e))?;
    let z_scale_factor = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read z scale factor", e))?;
    let x_offset = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read x offset", e))?;
    let y_offset = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read y offset", e))?;
    let z_offset = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read z offset", e))?;
    let max_x = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read max x", e))?;
    let min_x = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read min x", e))?;
    let max_y = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read max y", e))?;
    let min_y = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read min y", e))?;
    let max_z = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read max z", e))?;
    let min_z = reader
        .read_f64::<LittleEndian>()
        .map_err(|e| Error::io("read min z", e))?;
    reader
        .seek(SeekFrom::Start(235))
        .map_err(|e| Error::io("seek LAS 1.4 fields", e))?;
    let offset_to_first_evlr = reader
        .read_u64::<LittleEndian>()
        .map_err(|e| Error::io("read first EVLR offset", e))?;
    let number_of_evlrs = reader
        .read_u32::<LittleEndian>()
        .map_err(|e| Error::io("read EVLR count", e))?;
    if number_of_evlrs > MAX_EVLR_COUNT {
        return Err(Error::InvalidData(format!(
            "EVLR count {number_of_evlrs} exceeds max supported {MAX_EVLR_COUNT}"
        )));
    }
    if offset_to_first_evlr != 0 && offset_to_first_evlr > file_len {
        return Err(Error::InvalidData(format!(
            "first EVLR offset {offset_to_first_evlr} exceeds file length {file_len}"
        )));
    }
    let number_of_points = reader
        .read_u64::<LittleEndian>()
        .map_err(|e| Error::io("read point count", e))?;
    reader
        .seek(SeekFrom::Start(u64::from(header_size)))
        .map_err(|e| Error::io("seek after LAS header", e))?;
    Ok(LasHeader {
        point_data_record_format,
        point_data_record_length,
        offset_to_point_data,
        number_of_vlrs,
        x_scale_factor,
        y_scale_factor,
        z_scale_factor,
        x_offset,
        y_offset,
        z_offset,
        min_x,
        max_x,
        min_y,
        max_y,
        min_z,
        max_z,
        offset_to_first_evlr,
        number_of_evlrs,
        number_of_points,
    })
}

fn read_vlrs<R: Read + Seek>(
    reader: &mut R,
    count: u32,
    file_len: u64,
    section_end: u64,
) -> Result<Vec<Vlr>> {
    if count > MAX_VLR_COUNT {
        return Err(Error::InvalidData(format!(
            "VLR count {count} exceeds max supported {MAX_VLR_COUNT}"
        )));
    }
    if section_end > file_len {
        return Err(Error::InvalidData(format!(
            "VLR section end {section_end} exceeds file length {file_len}"
        )));
    }
    let mut vlrs = Vec::new();
    for index in 0..count {
        let header_offset = reader
            .stream_position()
            .map_err(|e| Error::io("record VLR offset", e))?;
        validate_range_in_file(header_offset, VLR_HEADER_BYTES, section_end, "VLR header")?;
        let _reserved = reader
            .read_u16::<LittleEndian>()
            .map_err(|e| Error::io("read VLR reserved", e))?;
        let mut user_id = [0u8; 16];
        reader
            .read_exact(&mut user_id)
            .map_err(|e| Error::io("read VLR user id", e))?;
        let record_id = reader
            .read_u16::<LittleEndian>()
            .map_err(|e| Error::io("read VLR record id", e))?;
        let record_length = reader
            .read_u16::<LittleEndian>()
            .map_err(|e| Error::io("read VLR length", e))?;
        let mut description = [0u8; 32];
        reader
            .read_exact(&mut description)
            .map_err(|e| Error::io("read VLR description", e))?;
        let data_offset = reader
            .stream_position()
            .map_err(|e| Error::io("record VLR data offset", e))?;
        let data_end = validate_range_in_file(
            data_offset,
            u64::from(record_length),
            section_end,
            "VLR data",
        )?;
        let user_id_str = trim_nul(&user_id).to_string();
        if should_store_vlr(&user_id_str, record_id) {
            let mut data = vec![0u8; usize::from(record_length)];
            reader
                .read_exact(&mut data)
                .map_err(|e| Error::io("read VLR data", e))?;
            vlrs.push(Vlr {
                user_id: user_id_str,
                record_id,
                data,
            });
        } else {
            reader
                .seek(SeekFrom::Start(data_end))
                .map_err(|e| Error::io("skip VLR data", e))?;
        }
        let actual_next = reader
            .stream_position()
            .map_err(|e| Error::io("record next VLR offset", e))?;
        if actual_next != data_end {
            return Err(Error::InvalidData(format!(
                "VLR {index} cursor at {actual_next}, expected {data_end}"
            )));
        }
    }
    Ok(vlrs)
}

fn should_store_vlr(user_id: &str, record_id: u16) -> bool {
    (user_id == "copc" && record_id == 1) || (user_id == "laszip encoded" && record_id == 22204)
}

fn read_evlr_refs<R: Read + Seek>(
    reader: &mut R,
    header: &LasHeader,
    file_len: u64,
) -> Result<Vec<EvlrRef>> {
    if header.offset_to_first_evlr == 0 || header.number_of_evlrs == 0 {
        return Ok(Vec::new());
    }
    if header.number_of_evlrs > MAX_EVLR_COUNT {
        return Err(Error::InvalidData(format!(
            "EVLR count {} exceeds max supported {}",
            header.number_of_evlrs, MAX_EVLR_COUNT
        )));
    }
    validate_range_in_file(
        header.offset_to_first_evlr,
        EVLR_HEADER_BYTES,
        file_len,
        "first EVLR header",
    )?;
    reader
        .seek(SeekFrom::Start(header.offset_to_first_evlr))
        .map_err(|e| Error::io("seek EVLRs", e))?;
    let mut evlrs = Vec::new();
    for index in 0..header.number_of_evlrs {
        let header_start = reader
            .stream_position()
            .map_err(|e| Error::io("record EVLR offset", e))?;
        validate_range_in_file(header_start, EVLR_HEADER_BYTES, file_len, "EVLR header")?;
        let _reserved = reader
            .read_u16::<LittleEndian>()
            .map_err(|e| Error::io("read EVLR reserved", e))?;
        let mut user_id = [0u8; 16];
        reader
            .read_exact(&mut user_id)
            .map_err(|e| Error::io("read EVLR user id", e))?;
        let record_id = reader
            .read_u16::<LittleEndian>()
            .map_err(|e| Error::io("read EVLR record id", e))?;
        let data_len = reader
            .read_u64::<LittleEndian>()
            .map_err(|e| Error::io("read EVLR length", e))?;
        let mut description = [0u8; 32];
        reader
            .read_exact(&mut description)
            .map_err(|e| Error::io("read EVLR description", e))?;
        let data_offset = reader
            .stream_position()
            .map_err(|e| Error::io("record EVLR data offset", e))?;
        evlrs.push(EvlrRef {
            user_id,
            record_id,
            data_offset,
        });
        let expected_next = validate_range_in_file(data_offset, data_len, file_len, "EVLR data")?;
        reader
            .seek(SeekFrom::Start(expected_next))
            .map_err(|e| Error::io("skip EVLR data", e))?;
        let actual_next = reader
            .stream_position()
            .map_err(|e| Error::io("record next EVLR offset", e))?;
        if actual_next != expected_next {
            return Err(Error::InvalidData(format!(
                "EVLR {index} cursor at {actual_next}, expected {expected_next}"
            )));
        }
    }
    Ok(evlrs)
}

fn trim_nul(bytes: &[u8]) -> &str {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    std::str::from_utf8(&bytes[..end]).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;

    use byteorder::{LittleEndian, WriteBytesExt};
    use copc_core::{EntryAvailability, HIERARCHY_ENTRY_BYTES};
    use laz::LazVlrBuilder;
    use std::io::{Cursor, Write};

    #[test]
    fn hierarchy_walk_loads_recursive_child_pages() {
        let mut fixture = Cursor::new(copc_with_child_hierarchy_page());
        let file = CopcFile::from_reader(&mut fixture).unwrap();
        let child_key = VoxelKey::root().child(3);
        let grandchild_key = child_key.child(5);

        assert_eq!(file.root_hierarchy().entries().len(), 2);
        assert!(file.root_hierarchy().entries()[1].is_child_page());

        let hierarchy = file.hierarchy();
        assert_eq!(hierarchy.len(), 3);
        assert_eq!(
            hierarchy
                .get(&VoxelKey::root())
                .unwrap()
                .availability()
                .unwrap(),
            EntryAvailability::PointData { point_count: 5 }
        );
        assert_eq!(
            hierarchy.get(&child_key).unwrap().availability().unwrap(),
            EntryAvailability::PointData { point_count: 4 }
        );
        assert_eq!(
            hierarchy
                .get(&grandchild_key)
                .unwrap()
                .availability()
                .unwrap(),
            EntryAvailability::PointData { point_count: 3 }
        );
        assert!(!hierarchy.values().any(|entry| entry.is_child_page()));

        let walk = file.hierarchy_walk();
        assert_eq!(walk.len(), hierarchy.len());
        assert_eq!(walk.iter().map(|entry| entry.point_count).sum::<i32>(), 12);
    }

    #[test]
    fn rejects_excessive_vlr_count_before_allocation() {
        let mut bytes = copc_with_child_hierarchy_page();
        put_u32(&mut bytes, 100, MAX_VLR_COUNT + 1);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("VLR count"));
    }

    #[test]
    fn rejects_excessive_evlr_count_before_allocation() {
        let mut bytes = copc_with_child_hierarchy_page();
        put_u32(&mut bytes, 243, MAX_EVLR_COUNT + 1);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("EVLR count"));
    }

    #[test]
    fn rejects_oversized_root_hierarchy_page_before_allocation() {
        let mut bytes = copc_with_child_hierarchy_page();
        let copc_info_data = usize::from(LAS_HEADER_SIZE_14) + VLR_HEADER_BYTES as usize;
        put_u64(
            &mut bytes,
            copc_info_data + 48,
            MAX_HIERARCHY_PAGE_BYTES + HIERARCHY_ENTRY_BYTES as u64,
        );

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("hierarchy page"));
        assert!(err.to_string().contains("max supported"));
    }

    #[test]
    fn rejects_child_hierarchy_page_outside_file() {
        let mut bytes = copc_with_child_hierarchy_page();
        let copc_info_data = usize::from(LAS_HEADER_SIZE_14) + VLR_HEADER_BYTES as usize;
        let root_hier_offset = read_u64(&bytes, copc_info_data + 40) as usize;
        let child_entry_offset_field = root_hier_offset + HIERARCHY_ENTRY_BYTES + 16;
        let outside_file = bytes.len() as u64 + 1;
        put_u64(&mut bytes, child_entry_offset_field, outside_file);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("child hierarchy page"));
        assert!(err.to_string().contains("exceeds file length"));
    }

    #[test]
    fn rejects_header_offsets_outside_file_before_allocation() {
        for (offset, value, expected) in [
            (94, u64::from(u16::MAX), "LAS header size"),
            (96, 1, "point data offset"),
            (96, u64::MAX, "point data offset"),
            (235, u64::MAX, "first EVLR offset"),
        ] {
            let mut bytes = copc_with_child_hierarchy_page();
            put_int(&mut bytes, offset, value);

            let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

            assert!(
                err.to_string().contains(expected),
                "expected {expected:?}, got {err}"
            );
        }
    }

    #[test]
    fn rejects_vlr_and_evlr_lengths_outside_file_before_allocation() {
        let mut bytes = copc_with_child_hierarchy_page();
        let first_vlr_length_field = usize::from(LAS_HEADER_SIZE_14) + 20;
        put_u16(&mut bytes, first_vlr_length_field, u16::MAX);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("VLR data"));
        assert!(err.to_string().contains("exceeds file length"));

        let mut bytes = copc_with_child_hierarchy_page();
        let evlr_start = read_u64(&bytes, 235) as usize;
        let evlr_length_field = evlr_start + 20;
        put_u64(&mut bytes, evlr_length_field, u64::MAX);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("EVLR data"));
        assert!(
            err.to_string().contains("overflow") || err.to_string().contains("exceeds file length")
        );
    }

    #[test]
    fn rejects_malformed_root_hierarchy_sizes() {
        for (root_hier_size, expected) in [
            (0, "empty"),
            (HIERARCHY_ENTRY_BYTES as u64 - 1, "not a multiple"),
        ] {
            let mut bytes = copc_with_child_hierarchy_page();
            let copc_info_data = usize::from(LAS_HEADER_SIZE_14) + VLR_HEADER_BYTES as usize;
            put_u64(&mut bytes, copc_info_data + 48, root_hier_size);

            let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

            assert!(
                err.to_string().contains(expected),
                "expected {expected:?}, got {err}"
            );
        }
    }

    #[test]
    fn rejects_invalid_hierarchy_entry_byte_sizes() {
        let mut bytes = copc_with_child_hierarchy_page();
        let copc_info_data = usize::from(LAS_HEADER_SIZE_14) + VLR_HEADER_BYTES as usize;
        let root_hier_offset = read_u64(&bytes, copc_info_data + 40) as usize;
        put_i32(&mut bytes, root_hier_offset + 24, 0);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("point data entry"));
        assert!(err.to_string().contains("invalid byte size"));

        let mut bytes = copc_with_child_hierarchy_page();
        let root_hier_offset = read_u64(&bytes, copc_info_data + 40) as usize;
        put_i32(&mut bytes, root_hier_offset + HIERARCHY_ENTRY_BYTES + 24, 0);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("child hierarchy page"));
        assert!(err.to_string().contains("invalid byte size"));
    }

    #[test]
    fn deep_child_page_chain_loads_without_stack_overflow() {
        // 200k chained pages would overflow the stack under recursive loading.
        let depth = 200_000;
        let mut fixture = Cursor::new(copc_with_deep_child_page_chain(depth));

        let file = CopcFile::from_reader(&mut fixture).unwrap();

        assert_eq!(file.hierarchy().len(), depth);
    }

    #[test]
    fn rejects_hierarchy_point_total_exceeding_header_count() {
        // Entries sum to 12 points; lower the header count to 11.
        let mut bytes = copc_with_child_hierarchy_page();
        put_u64(&mut bytes, 247, 11);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("hierarchy point total"));
    }

    #[test]
    fn rejects_huge_claimed_point_count_before_allocation() {
        let mut bytes = copc_with_child_hierarchy_page();
        let copc_info_data = usize::from(LAS_HEADER_SIZE_14) + VLR_HEADER_BYTES as usize;
        let root_hier_offset = read_u64(&bytes, copc_info_data + 40) as usize;
        put_i32(&mut bytes, root_hier_offset + 28, i32::MAX);

        let err = CopcFile::from_reader(&mut Cursor::new(bytes)).unwrap_err();

        assert!(err.to_string().contains("hierarchy point total"));
    }

    #[test]
    fn truncated_inputs_fail_without_panicking() {
        let bytes = copc_with_child_hierarchy_page();
        for len in [
            0,
            1,
            4,
            128,
            usize::from(LAS_HEADER_SIZE_14) - 1,
            usize::from(LAS_HEADER_SIZE_14),
            bytes.len() / 2,
            bytes.len() - 1,
        ] {
            let truncated = bytes[..len].to_vec();

            let err = CopcFile::from_reader(&mut Cursor::new(truncated)).unwrap_err();

            assert!(
                !err.to_string().is_empty(),
                "truncated input length {len} produced an empty error"
            );
        }
    }

    fn copc_with_child_hierarchy_page() -> Vec<u8> {
        let mut laz_vlr_bytes = Vec::new();
        LazVlrBuilder::default()
            .with_point_format(6, 0)
            .unwrap()
            .with_variable_chunk_size()
            .build()
            .write_to(&mut laz_vlr_bytes)
            .unwrap();

        let offset_to_point_data = u32::from(LAS_HEADER_SIZE_14)
            + (54 + copc_core::info::COPC_INFO_BYTES as u32)
            + (54 + laz_vlr_bytes.len() as u32);
        let root_point_offset = u64::from(offset_to_point_data);
        let child_point_offset = root_point_offset + 100;
        let grandchild_point_offset = child_point_offset + 200;
        let evlr_start = grandchild_point_offset + 220;
        let root_hier_offset = evlr_start + 60;
        let root_hier_size = (2 * HIERARCHY_ENTRY_BYTES) as u64;
        let child_page_offset = root_hier_offset + root_hier_size;

        let child_key = VoxelKey::root().child(3);
        let grandchild_key = child_key.child(5);
        let child_page = HierarchyPage::new(vec![
            Entry {
                key: child_key,
                offset: child_point_offset,
                byte_size: 200,
                point_count: 4,
            },
            Entry {
                key: grandchild_key,
                offset: grandchild_point_offset,
                byte_size: 220,
                point_count: 3,
            },
        ]);
        let child_page_bytes = child_page.write_le_bytes().unwrap();
        let root_page = HierarchyPage::new(vec![
            Entry {
                key: VoxelKey::root(),
                offset: root_point_offset,
                byte_size: 100,
                point_count: 5,
            },
            Entry {
                key: child_key,
                offset: child_page_offset,
                byte_size: child_page_bytes.len() as i32,
                point_count: -1,
            },
        ]);
        let root_page_bytes = root_page.write_le_bytes().unwrap();

        let info = CopcInfo {
            center: (0.0, 0.0, 0.0),
            halfsize: 10.0,
            spacing: 1.0,
            root_hier_offset,
            root_hier_size,
            gpstime_min: 0.0,
            gpstime_max: 0.0,
        };

        let mut out = Vec::new();
        write_las_header(&mut out, offset_to_point_data, evlr_start, 12);
        write_vlr(&mut out, "copc", 1, &info.write_le_bytes(), "COPC info");
        write_vlr(
            &mut out,
            "laszip encoded",
            22204,
            &laz_vlr_bytes,
            "http://laszip.org",
        );
        assert_eq!(out.len(), offset_to_point_data as usize);
        out.resize(evlr_start as usize, 0);

        write_evlr_header(
            &mut out,
            "copc",
            1000,
            root_page_bytes.len() as u64,
            "COPC hierarchy",
        );
        assert_eq!(out.len() as u64, root_hier_offset);
        out.extend_from_slice(&root_page_bytes);
        assert_eq!(out.len() as u64, child_page_offset);
        out.extend_from_slice(&child_page_bytes);
        out
    }

    /// A COPC file whose hierarchy is a linear chain of `depth` single-entry
    /// pages, each pointing at the next; the final page holds one empty entry.
    fn copc_with_deep_child_page_chain(depth: usize) -> Vec<u8> {
        assert!(depth >= 2);
        let mut laz_vlr_bytes = Vec::new();
        LazVlrBuilder::default()
            .with_point_format(6, 0)
            .unwrap()
            .with_variable_chunk_size()
            .build()
            .write_to(&mut laz_vlr_bytes)
            .unwrap();

        let offset_to_point_data = u32::from(LAS_HEADER_SIZE_14)
            + (54 + copc_core::info::COPC_INFO_BYTES as u32)
            + (54 + laz_vlr_bytes.len() as u32);
        let evlr_start = u64::from(offset_to_point_data);
        let root_hier_offset = evlr_start + 60;
        let page_offset = |page: usize| root_hier_offset + (page * HIERARCHY_ENTRY_BYTES) as u64;

        let info = CopcInfo {
            center: (0.0, 0.0, 0.0),
            halfsize: 10.0,
            spacing: 1.0,
            root_hier_offset,
            root_hier_size: HIERARCHY_ENTRY_BYTES as u64,
            gpstime_min: 0.0,
            gpstime_max: 0.0,
        };

        let mut out = Vec::new();
        write_las_header(&mut out, offset_to_point_data, evlr_start, 0);
        write_vlr(&mut out, "copc", 1, &info.write_le_bytes(), "COPC info");
        write_vlr(
            &mut out,
            "laszip encoded",
            22204,
            &laz_vlr_bytes,
            "http://laszip.org",
        );
        assert_eq!(out.len(), offset_to_point_data as usize);
        write_evlr_header(
            &mut out,
            "copc",
            1000,
            (depth * HIERARCHY_ENTRY_BYTES) as u64,
            "COPC hierarchy",
        );
        assert_eq!(out.len() as u64, root_hier_offset);
        for page in 0..depth {
            let entry = if page + 1 < depth {
                Entry {
                    key: VoxelKey {
                        level: (page + 1) as i32,
                        x: 0,
                        y: 0,
                        z: 0,
                    },
                    offset: page_offset(page + 1),
                    byte_size: HIERARCHY_ENTRY_BYTES as i32,
                    point_count: -1,
                }
            } else {
                Entry {
                    key: VoxelKey {
                        level: depth as i32,
                        x: 0,
                        y: 0,
                        z: 0,
                    },
                    offset: 0,
                    byte_size: 0,
                    point_count: 0,
                }
            };
            let mut entry_bytes = [0u8; HIERARCHY_ENTRY_BYTES];
            entry.write_le(&mut entry_bytes).unwrap();
            out.extend_from_slice(&entry_bytes);
        }
        out
    }

    fn write_las_header(
        out: &mut Vec<u8>,
        offset_to_point_data: u32,
        evlr_start: u64,
        point_count: u64,
    ) {
        out.resize(usize::from(LAS_HEADER_SIZE_14), 0);
        out[0..4].copy_from_slice(b"LASF");
        out[24] = 1;
        out[25] = 4;
        put_u16(out, 94, LAS_HEADER_SIZE_14);
        put_u32(out, 96, offset_to_point_data);
        put_u32(out, 100, 2);
        out[104] = 6 | 0x80;
        put_u16(out, 105, 30);
        put_f64(out, 131, 0.001);
        put_f64(out, 139, 0.001);
        put_f64(out, 147, 0.001);
        put_f64(out, 155, 0.0);
        put_f64(out, 163, 0.0);
        put_f64(out, 171, 0.0);
        put_f64(out, 179, 10.0);
        put_f64(out, 187, -10.0);
        put_f64(out, 195, 10.0);
        put_f64(out, 203, -10.0);
        put_f64(out, 211, 10.0);
        put_f64(out, 219, -10.0);
        put_u64(out, 235, evlr_start);
        put_u32(out, 243, 1);
        put_u64(out, 247, point_count);
    }

    fn write_vlr(out: &mut Vec<u8>, user_id: &str, record_id: u16, data: &[u8], desc: &str) {
        out.write_u16::<LittleEndian>(0).unwrap();
        out.write_all(&padded(user_id.as_bytes(), 16)).unwrap();
        out.write_u16::<LittleEndian>(record_id).unwrap();
        out.write_u16::<LittleEndian>(data.len() as u16).unwrap();
        out.write_all(&padded(desc.as_bytes(), 32)).unwrap();
        out.write_all(data).unwrap();
    }

    fn write_evlr_header(
        out: &mut Vec<u8>,
        user_id: &str,
        record_id: u16,
        data_len: u64,
        desc: &str,
    ) {
        out.write_u16::<LittleEndian>(0).unwrap();
        out.write_all(&padded(user_id.as_bytes(), 16)).unwrap();
        out.write_u16::<LittleEndian>(record_id).unwrap();
        out.write_u64::<LittleEndian>(data_len).unwrap();
        out.write_all(&padded(desc.as_bytes(), 32)).unwrap();
    }

    fn padded(bytes: &[u8], len: usize) -> Vec<u8> {
        let mut out = vec![0u8; len];
        let count = bytes.len().min(len);
        out[..count].copy_from_slice(&bytes[..count]);
        out
    }

    fn put_u16(out: &mut [u8], offset: usize, value: u16) {
        out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn put_u32(out: &mut [u8], offset: usize, value: u32) {
        out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn put_u64(out: &mut [u8], offset: usize, value: u64) {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }

    fn put_i32(out: &mut [u8], offset: usize, value: i32) {
        out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn put_int(out: &mut [u8], offset: usize, value: u64) {
        match offset {
            94 => put_u16(out, offset, value as u16),
            96 => put_u32(out, offset, value as u32),
            235 => put_u64(out, offset, value),
            _ => unreachable!("unexpected integer offset"),
        }
    }

    fn read_u64(bytes: &[u8], offset: usize) -> u64 {
        u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
    }

    fn put_f64(out: &mut [u8], offset: usize, value: f64) {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
