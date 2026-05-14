//! Pure-Rust COPC reader.
//!
//! The initial implementation parses LAS/COPC headers, the COPC info VLR, and
//! the root hierarchy EVLR. Chunked-LAZ point iteration will be layered on top
//! of the same parsed metadata.

#![forbid(unsafe_code)]

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use byteorder::{LittleEndian, ReadBytesExt};
use copc_core::{CopcInfo, Entry, Error, HierarchyPage, Result};

const LAS_HEADER_SIZE_14: u16 = 375;
/// A parsed COPC file.
#[derive(Debug, Clone)]
pub struct CopcFile {
    header: LasHeader,
    copc_info: CopcInfo,
    root_hierarchy: HierarchyPage,
}

/// Minimal LAS header fields needed by COPC callers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LasHeader {
    pub point_data_record_format: u8,
    pub point_data_record_length: u16,
    pub offset_to_point_data: u32,
    pub number_of_vlrs: u32,
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
        let hierarchy_len = usize::try_from(copc_info.root_hier_size)
            .map_err(|_| Error::InvalidData("root hierarchy is too large".into()))?;
        let mut hierarchy_bytes = vec![0u8; hierarchy_len];
        reader
            .seek(SeekFrom::Start(copc_info.root_hier_offset))
            .map_err(|e| Error::io("seek root hierarchy", e))?;
        reader
            .read_exact(&mut hierarchy_bytes)
            .map_err(|e| Error::io("read root hierarchy", e))?;
        let root_hierarchy = HierarchyPage::from_le_bytes(&hierarchy_bytes)?;
        Ok(Self {
            header,
            copc_info,
            root_hierarchy,
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

    /// Return the currently parsed hierarchy entries.
    ///
    /// This initial implementation returns the root hierarchy page. Recursive
    /// child-page loading belongs with the chunked-LAZ reader work.
    pub fn hierarchy_walk(&self) -> Vec<Entry> {
        self.root_hierarchy.entries().to_vec()
    }
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
        .seek(SeekFrom::Start(179))
        .map_err(|e| Error::io("seek LAS bounds", e))?;
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
