//! Lazy, range-request-driven COPC reading.
//!
//! [`CopcRangeReader`] fetches only what a query needs from a [`RangeRead`]
//! source: the LAS header and VLRs at open, hierarchy pages on demand as
//! queries touch their subtrees, and point chunks via coalesced range reads.

use std::collections::{BTreeMap, HashSet};
use std::io::{Read, Seek, SeekFrom};

use copc_core::{
    ColumnSelection, CopcInfo, Entry, EntryAvailability, Error, HierarchyPage, LasColumnBatch,
    Result, VoxelKey,
};
use las::Point;
use laz::LazVlr;

use crate::points::{
    decode_chunk_points, select_point_chunks_from, selected_column_builders,
    total_candidate_points, voxel_bounds, BoundsSelection, ChunkColumnDecoder, PointQuery,
    MAX_INITIAL_COLUMN_RESERVE_POINTS,
};
use crate::range_read::RangeRead;
use crate::{
    extract_required_vlrs, point_format_for, read_las_header, read_vlrs, transforms_for,
    validate_hierarchy_entry, validate_range_in_file, HierarchyReadLimits, LasHeader,
    LAS_HEADER_SIZE_14,
};

/// Adjacent chunk ranges closer than this are fetched with one request.
const RANGE_COALESCE_GAP_BYTES: u64 = 64 * 1024;
/// Cap on the VLR section fetched eagerly at open.
const MAX_VLR_SECTION_BYTES: u64 = 64 * 1024 * 1024;

/// Presents a fetched byte section at its absolute file offsets so the
/// shared header/VLR parsers can run over it unchanged.
struct SectionReader {
    base: u64,
    data: Vec<u8>,
    position: u64,
}

impl SectionReader {
    fn new(base: u64, data: Vec<u8>) -> Self {
        Self {
            base,
            data,
            position: base,
        }
    }
}

impl Read for SectionReader {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let start = self.position.checked_sub(self.base).ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "read before fetched section",
            )
        })? as usize;
        if start > self.data.len() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "read past fetched section",
            ));
        }
        let available = &self.data[start..];
        let take = available.len().min(buf.len());
        buf[..take].copy_from_slice(&available[..take]);
        self.position += take as u64;
        Ok(take)
    }
}

impl Seek for SectionReader {
    fn seek(&mut self, pos: SeekFrom) -> std::io::Result<u64> {
        let target = match pos {
            SeekFrom::Start(offset) => Some(offset),
            SeekFrom::Current(delta) => self.position.checked_add_signed(delta),
            SeekFrom::End(delta) => (self.base + self.data.len() as u64).checked_add_signed(delta),
        }
        .ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, "seek out of range")
        })?;
        self.position = target;
        Ok(self.position)
    }
}

/// Lazy COPC reader over a byte-range source.
///
/// Opening fetches the LAS header, the VLR section, and the root hierarchy
/// page. Hierarchy pages for deeper subtrees are fetched only when a query's
/// LOD/bounds selection can reach them, and point chunks are fetched with
/// coalesced range requests.
pub struct CopcRangeReader<S: RangeRead> {
    source: S,
    file_len: u64,
    header: LasHeader,
    copc_info: CopcInfo,
    laszip_vlr: LazVlr,
    hierarchy: BTreeMap<VoxelKey, Entry>,
    pending_pages: Vec<Entry>,
    visited_pages: HashSet<(u64, u64)>,
    limits: HierarchyReadLimits,
    loaded_point_total: u64,
}

impl<S: RangeRead> CopcRangeReader<S> {
    pub fn open(mut source: S) -> Result<Self> {
        let file_len = source.len()?;
        if file_len < u64::from(LAS_HEADER_SIZE_14) {
            return Err(Error::InvalidData(format!(
                "file is {file_len} bytes; COPC requires at least {LAS_HEADER_SIZE_14}"
            )));
        }
        let mut header_bytes = vec![0u8; usize::from(LAS_HEADER_SIZE_14)];
        source.read_range(0, &mut header_bytes)?;
        let mut header_section = SectionReader::new(0, header_bytes);
        let header = read_las_header(&mut header_section, file_len)?;
        let header_size = header_section
            .stream_position()
            .map_err(|e| Error::io("record header size", e))?;

        let vlr_section_len = u64::from(header.offset_to_point_data)
            .checked_sub(header_size)
            .ok_or_else(|| Error::InvalidData("VLR section precedes LAS header end".into()))?;
        if vlr_section_len > MAX_VLR_SECTION_BYTES {
            return Err(Error::InvalidData(format!(
                "VLR section is {vlr_section_len} bytes, max supported is {MAX_VLR_SECTION_BYTES}"
            )));
        }
        let mut vlr_bytes = vec![0u8; vlr_section_len as usize];
        source.read_range(header_size, &mut vlr_bytes)?;
        let mut vlr_section = SectionReader::new(header_size, vlr_bytes);
        let vlrs = read_vlrs(
            &mut vlr_section,
            header.number_of_vlrs,
            file_len,
            u64::from(header.offset_to_point_data),
        )?;
        let (copc_info, laszip_vlr) = extract_required_vlrs(&vlrs)?;

        let mut reader = Self {
            source,
            file_len,
            header,
            copc_info,
            laszip_vlr,
            hierarchy: BTreeMap::new(),
            pending_pages: Vec::new(),
            visited_pages: HashSet::new(),
            limits: HierarchyReadLimits::default(),
            loaded_point_total: 0,
        };
        let root = Entry {
            key: VoxelKey::root(),
            offset: reader.copc_info.root_hier_offset,
            byte_size: i32::try_from(reader.copc_info.root_hier_size)
                .map_err(|_| Error::InvalidData("root hierarchy size exceeds i32".into()))?,
            point_count: -1,
        };
        reader.load_page(root)?;
        Ok(reader)
    }

    pub fn header(&self) -> &LasHeader {
        &self.header
    }

    pub fn copc_info(&self) -> &CopcInfo {
        &self.copc_info
    }

    /// Point-data hierarchy entries matching `query`, loading any hierarchy
    /// pages the query can reach that have not been fetched yet.
    pub fn hierarchy_for(&mut self, query: PointQuery) -> Result<Vec<Entry>> {
        self.ensure_hierarchy_for(query)?;
        select_point_chunks_from(self.hierarchy.values(), &self.copc_info, query)
    }

    /// Materialized column read over the chunks selected by `query`.
    pub fn read_columns(
        &mut self,
        query: PointQuery,
        selection: ColumnSelection,
    ) -> Result<LasColumnBatch> {
        let chunks = self.hierarchy_for(query)?;
        let point_format = point_format_for(&self.header)?;
        let bounds = match query.bounds {
            BoundsSelection::All => None,
            BoundsSelection::Within(bounds) => Some(bounds),
        };
        let decoder = ChunkColumnDecoder::new(
            self.laszip_vlr.clone(),
            point_format,
            transforms_for(&self.header),
            bounds,
        )?;
        let capacity = match bounds {
            Some(_) => 0,
            None => total_candidate_points(&chunks)?.min(MAX_INITIAL_COLUMN_RESERVE_POINTS),
        };
        let mut columns = selected_column_builders(point_format, selection, capacity)?;
        let mut accepted_points = 0usize;
        for group in coalesce_chunks(&chunks, RANGE_COALESCE_GAP_BYTES) {
            let bytes = self.fetch_range(group.start, group.end)?;
            for entry in group.entries {
                let (slice, points_in_chunk) = chunk_slice(&bytes, group.start, entry)?;
                accepted_points +=
                    decoder.decode_into(slice, points_in_chunk, &mut columns, None)?;
            }
        }
        let batch = LasColumnBatch {
            len: accepted_points,
            columns,
        };
        batch.validate()?;
        Ok(batch)
    }

    /// Materialized row read over the chunks selected by `query`.
    pub fn read_points(&mut self, query: PointQuery) -> Result<Vec<Point>> {
        let chunks = self.hierarchy_for(query)?;
        let point_format = point_format_for(&self.header)?;
        let transforms = transforms_for(&self.header);
        let bounds = match query.bounds {
            BoundsSelection::All => None,
            BoundsSelection::Within(bounds) => Some(bounds),
        };
        let capacity = match bounds {
            Some(_) => 0,
            None => total_candidate_points(&chunks)?.min(MAX_INITIAL_COLUMN_RESERVE_POINTS),
        };
        let mut points = Vec::with_capacity(capacity);
        for group in coalesce_chunks(&chunks, RANGE_COALESCE_GAP_BYTES) {
            let bytes = self.fetch_range(group.start, group.end)?;
            for entry in group.entries {
                let (slice, points_in_chunk) = chunk_slice(&bytes, group.start, entry)?;
                decode_chunk_points(
                    slice,
                    points_in_chunk,
                    &self.laszip_vlr,
                    &point_format,
                    &transforms,
                    bounds,
                    &mut points,
                )?;
            }
        }
        Ok(points)
    }

    fn fetch_range(&mut self, start: u64, end: u64) -> Result<Vec<u8>> {
        let len = usize::try_from(
            end.checked_sub(start)
                .ok_or_else(|| Error::InvalidData("chunk range end precedes its start".into()))?,
        )
        .map_err(|_| Error::InvalidData("chunk range exceeds usize".into()))?;
        let mut bytes = vec![0u8; len];
        self.source.read_range(start, &mut bytes)?;
        Ok(bytes)
    }

    /// Loads every pending hierarchy page whose subtree the query can reach.
    fn ensure_hierarchy_for(&mut self, query: PointQuery) -> Result<()> {
        loop {
            let mut relevant = Vec::new();
            let mut rest = Vec::new();
            for entry in self.pending_pages.drain(..) {
                if page_may_match(entry.key, &self.copc_info, query)? {
                    relevant.push(entry);
                } else {
                    rest.push(entry);
                }
            }
            self.pending_pages = rest;
            if relevant.is_empty() {
                return Ok(());
            }
            for entry in relevant {
                self.load_page(entry)?;
            }
        }
    }

    fn load_page(&mut self, page_entry: Entry) -> Result<()> {
        let byte_size = u64::try_from(page_entry.byte_size).map_err(|_| {
            Error::InvalidData(format!(
                "child hierarchy page {:?} has negative byte size {}",
                page_entry.key, page_entry.byte_size
            ))
        })?;
        if !self.visited_pages.insert((page_entry.offset, byte_size)) {
            return Ok(());
        }
        if byte_size == 0 || byte_size % copc_core::HIERARCHY_ENTRY_BYTES as u64 != 0 {
            return Err(Error::InvalidData(format!(
                "hierarchy page is {byte_size} bytes, not a multiple of {}",
                copc_core::HIERARCHY_ENTRY_BYTES
            )));
        }
        self.limits.add_page(byte_size)?;
        validate_range_in_file(
            page_entry.offset,
            byte_size,
            self.file_len,
            "hierarchy page",
        )?;
        let bytes = self.fetch_range(page_entry.offset, page_entry.offset + byte_size)?;
        let page = HierarchyPage::from_le_bytes(&bytes)?;
        for entry in page.entries().iter().copied() {
            validate_hierarchy_entry(entry, self.file_len)?;
            if let Some(previous) = self.hierarchy.insert(entry.key, entry) {
                if let EntryAvailability::PointData { point_count } = previous.availability()? {
                    self.loaded_point_total -= u64::from(point_count);
                }
            }
            if let EntryAvailability::PointData { point_count } = entry.availability()? {
                self.loaded_point_total += u64::from(point_count);
            }
            if entry.is_child_page() {
                self.pending_pages.push(entry);
            }
        }
        if self.loaded_point_total > self.header.number_of_points {
            return Err(Error::InvalidData(format!(
                "hierarchy point total {} exceeds LAS header point count {}",
                self.loaded_point_total, self.header.number_of_points
            )));
        }
        Ok(())
    }
}

/// Whether the page rooted at `key` can contain entries matching `query`.
/// Page entries all live in `key`'s subtree, so their levels are at least
/// `key.level` and their bounds nest inside `key`'s voxel.
fn page_may_match(key: VoxelKey, info: &CopcInfo, query: PointQuery) -> Result<bool> {
    let (_, level_max) = crate::points::level_range(query.lod, info)?;
    if key.level >= level_max {
        return Ok(false);
    }
    if let BoundsSelection::Within(bounds) = query.bounds {
        if key.level >= 0 && key.x >= 0 && key.y >= 0 && key.z >= 0 {
            return Ok(voxel_bounds(key, info)?.intersects(bounds));
        }
    }
    Ok(true)
}

struct ChunkGroup {
    start: u64,
    end: u64,
    entries: Vec<Entry>,
}

/// Groups offset-sorted chunks so nearby ranges are fetched with one request.
fn coalesce_chunks(chunks: &[Entry], gap: u64) -> Vec<ChunkGroup> {
    let mut groups: Vec<ChunkGroup> = Vec::new();
    for entry in chunks {
        let start = entry.offset;
        let end = entry.offset + entry.byte_size.max(0) as u64;
        match groups.last_mut() {
            Some(group) if start <= group.end.saturating_add(gap) => {
                group.end = group.end.max(end);
                group.entries.push(*entry);
            }
            _ => groups.push(ChunkGroup {
                start,
                end,
                entries: vec![*entry],
            }),
        }
    }
    groups
}

fn chunk_slice(group_bytes: &[u8], group_start: u64, entry: Entry) -> Result<(&[u8], usize)> {
    let points_in_chunk = usize::try_from(entry.point_count).map_err(|_| {
        Error::InvalidData(format!(
            "negative point count {} for {:?}",
            entry.point_count, entry.key
        ))
    })?;
    let offset = usize::try_from(entry.offset - group_start)
        .map_err(|_| Error::InvalidData("chunk offset exceeds usize".into()))?;
    let byte_size = usize::try_from(entry.byte_size).map_err(|_| {
        Error::InvalidData(format!(
            "invalid byte size {} for {:?}",
            entry.byte_size, entry.key
        ))
    })?;
    let end = offset
        .checked_add(byte_size)
        .filter(|end| *end <= group_bytes.len())
        .ok_or_else(|| Error::InvalidData("chunk range exceeds fetched group".into()))?;
    Ok((&group_bytes[offset..end], points_in_chunk))
}
