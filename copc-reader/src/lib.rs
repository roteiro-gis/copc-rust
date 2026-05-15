//! Pure-Rust COPC reader.
//!
//! Parses LAS/COPC metadata and exposes chunked-LAZ point iteration over COPC
//! hierarchy entries.

#![forbid(unsafe_code)]

mod points;

use std::collections::{BTreeMap, HashSet};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt};
use copc_core::{CopcInfo, Entry, Error, HierarchyPage, Result, VoxelKey};
use las::{Transform, Vector};
use laz::LazVlr;

pub use points::{BoundsSelection, CopcReader, LodSelection, PointIter, PointQuery};

const LAS_HEADER_SIZE_14: u16 = 375;
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
        let header = read_las_header(reader)?;
        let vlrs = read_vlrs(reader, header.number_of_vlrs)?;
        let copc_info_vlr = vlrs
            .iter()
            .find(|vlr| vlr.user_id == "copc" && vlr.record_id == 1)
            .ok_or_else(|| Error::InvalidData("missing COPC info VLR".into()))?;
        let copc_info = CopcInfo::from_le_bytes(&copc_info_vlr.data)?;
        let laszip_vlr = vlrs
            .iter()
            .find(|vlr| vlr.user_id == "laszip encoded" && vlr.record_id == 22204)
            .map(|vlr| {
                LazVlr::read_from(vlr.data.as_slice()).map_err(|e| Error::Las(e.to_string()))
            })
            .transpose()?
            .ok_or_else(|| Error::InvalidData("missing LASzip VLR".into()))?;
        let evlrs = read_evlr_refs(reader, &header)?;
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
        let root_hierarchy =
            read_hierarchy_page_at(reader, copc_info.root_hier_offset, copc_info.root_hier_size)?;
        let mut hierarchy = BTreeMap::new();
        let mut visited_pages = HashSet::new();
        visited_pages.insert((copc_info.root_hier_offset, copc_info.root_hier_size));
        insert_hierarchy_page(reader, &root_hierarchy, &mut hierarchy, &mut visited_pages)?;
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
        let format_id = self.header.point_data_record_format & 0x7F;
        let mut format =
            las::point::Format::new(format_id).map_err(|e| Error::Las(e.to_string()))?;
        let base_len = format.len();
        if self.header.point_data_record_length < base_len {
            return Err(Error::InvalidData(format!(
                "point record length {} is smaller than point format {} base length {}",
                self.header.point_data_record_length, format_id, base_len
            )));
        }
        format.extra_bytes = self.header.point_data_record_length - base_len;
        Ok(format)
    }

    pub(crate) fn transforms(&self) -> Vector<Transform> {
        Vector {
            x: Transform {
                scale: self.header.x_scale_factor,
                offset: self.header.x_offset,
            },
            y: Transform {
                scale: self.header.y_scale_factor,
                offset: self.header.y_offset,
            },
            z: Transform {
                scale: self.header.z_scale_factor,
                offset: self.header.z_offset,
            },
        }
    }
}

impl LasHeader {
    pub fn number_of_points(&self) -> u64 {
        self.number_of_points
    }
}

fn read_hierarchy_page_at<R: Read + Seek>(
    reader: &mut R,
    offset: u64,
    byte_size: u64,
) -> Result<HierarchyPage> {
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

fn insert_hierarchy_page<R: Read + Seek>(
    reader: &mut R,
    page: &HierarchyPage,
    hierarchy: &mut BTreeMap<VoxelKey, Entry>,
    visited_pages: &mut HashSet<(u64, u64)>,
) -> Result<()> {
    for entry in page.entries().iter().copied() {
        hierarchy.insert(entry.key, entry);
    }
    for entry in page.entries().iter().copied().filter(|e| e.is_child_page()) {
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
        if visited_pages.insert((entry.offset, byte_size)) {
            let child_page = read_hierarchy_page_at(reader, entry.offset, byte_size)?;
            insert_hierarchy_page(reader, &child_page, hierarchy, visited_pages)?;
        }
    }
    Ok(())
}

fn read_las_header<R: Read + Seek>(reader: &mut R) -> Result<LasHeader> {
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
    let offset_to_point_data = reader
        .read_u32::<LittleEndian>()
        .map_err(|e| Error::io("read point data offset", e))?;
    let number_of_vlrs = reader
        .read_u32::<LittleEndian>()
        .map_err(|e| Error::io("read VLR count", e))?;
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

fn read_vlrs<R: Read>(reader: &mut R, count: u32) -> Result<Vec<Vlr>> {
    let mut vlrs = Vec::with_capacity(count as usize);
    for _ in 0..count {
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
        let mut data = vec![0u8; usize::from(record_length)];
        reader
            .read_exact(&mut data)
            .map_err(|e| Error::io("read VLR data", e))?;
        vlrs.push(Vlr {
            user_id: trim_nul(&user_id).to_string(),
            record_id,
            data,
        });
    }
    Ok(vlrs)
}

fn read_evlr_refs<R: Read + Seek>(reader: &mut R, header: &LasHeader) -> Result<Vec<EvlrRef>> {
    if header.offset_to_first_evlr == 0 || header.number_of_evlrs == 0 {
        return Ok(Vec::new());
    }
    reader
        .seek(SeekFrom::Start(header.offset_to_first_evlr))
        .map_err(|e| Error::io("seek EVLRs", e))?;
    let mut evlrs = Vec::with_capacity(header.number_of_evlrs as usize);
    for _ in 0..header.number_of_evlrs {
        let _header_start = reader
            .stream_position()
            .map_err(|e| Error::io("record EVLR offset", e))?;
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
        reader
            .seek(SeekFrom::Current(i64::try_from(data_len).map_err(
                |_| Error::InvalidData("EVLR length exceeds seek range".into()),
            )?))
            .map_err(|e| Error::io("skip EVLR data", e))?;
        let expected_next = data_offset + data_len;
        let actual_next = reader
            .stream_position()
            .map_err(|e| Error::io("record next EVLR offset", e))?;
        if actual_next != expected_next {
            return Err(Error::InvalidData(format!(
                "EVLR cursor at {actual_next}, expected {expected_next}"
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
        let evlr_start = u64::from(offset_to_point_data);
        let root_hier_offset = evlr_start + 60;
        let root_hier_size = (2 * HIERARCHY_ENTRY_BYTES) as u64;
        let child_page_offset = root_hier_offset + root_hier_size;

        let child_key = VoxelKey::root().child(3);
        let grandchild_key = child_key.child(5);
        let child_page = HierarchyPage::new(vec![
            Entry {
                key: child_key,
                offset: 2_000,
                byte_size: 200,
                point_count: 4,
            },
            Entry {
                key: grandchild_key,
                offset: 2_200,
                byte_size: 220,
                point_count: 3,
            },
        ]);
        let child_page_bytes = child_page.write_le_bytes().unwrap();
        let root_page = HierarchyPage::new(vec![
            Entry {
                key: VoxelKey::root(),
                offset: 1_000,
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

    fn put_f64(out: &mut [u8], offset: usize, value: f64) {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
