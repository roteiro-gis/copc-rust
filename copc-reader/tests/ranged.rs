//! CopcRangeReader behavior: request accounting, lazy hierarchy loading,
//! range coalescing, and equivalence with the seek-based reader.

use std::sync::{Arc, Mutex};

use copc_core::{Bounds, ColumnData, ColumnSelection, CopcInfo, Entry, HierarchyPage, VoxelKey};
use copc_reader::{
    BoundsSelection, CopcRangeReader, CopcReader, LasColumnBatch, LasDimension, LodSelection,
    PointQuery, RangeRead,
};
use copc_writer::{
    write_source, CopcPointFields, CopcPointSource, CopcWriteMetadata, CopcWriterParams,
};

/// In-memory range source that records every requested range.
#[derive(Clone)]
struct RecordingSource {
    data: Arc<Vec<u8>>,
    requests: Arc<Mutex<Vec<(u64, u64)>>>,
}

impl RecordingSource {
    fn new(data: Vec<u8>) -> Self {
        Self {
            data: Arc::new(data),
            requests: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn requests(&self) -> Vec<(u64, u64)> {
        self.requests.lock().unwrap().clone()
    }

    fn clear_requests(&self) {
        self.requests.lock().unwrap().clear();
    }
}

impl RangeRead for RecordingSource {
    fn len(&mut self) -> copc_core::Result<u64> {
        Ok(self.data.len() as u64)
    }

    fn read_range(&mut self, offset: u64, buf: &mut [u8]) -> copc_core::Result<()> {
        self.requests
            .lock()
            .unwrap()
            .push((offset, buf.len() as u64));
        let start = usize::try_from(offset).unwrap();
        let end = start + buf.len();
        assert!(
            end <= self.data.len(),
            "range {start}..{end} exceeds source length {}",
            self.data.len()
        );
        buf.copy_from_slice(&self.data[start..end]);
        Ok(())
    }
}

struct VecSource {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for VecSource {
    fn len(&self) -> usize {
        self.points.len()
    }

    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        let p = &self.points[index];
        (p.x, p.y, p.z)
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> copc_core::Result<()> {
        out.clone_from(&self.points[index]);
        Ok(())
    }
}

fn synthetic_copc_bytes(count: u32) -> Vec<u8> {
    let mut points = Vec::new();
    for i in 0..count {
        let f = f64::from(i);
        points.push(CopcPointFields {
            x: (f * 37.0) % 251.0,
            y: (f * 53.0) % 257.0,
            z: (f * 71.0) % 63.0,
            intensity: (i % 65_536) as u16,
            return_number: 1,
            number_of_returns: 1,
            gps_time: 1.0e9 + f,
            ..CopcPointFields::default()
        });
    }
    let bounds = points.iter().fold(
        Bounds::point(points[0].x, points[0].y, points[0].z),
        |mut bounds, point| {
            bounds.extend(point.x, point.y, point.z);
            bounds
        },
    );
    let source = VecSource { points };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ranged.copc.laz");
    write_source(
        &path,
        &source,
        false,
        bounds,
        &CopcWriterParams::new(500),
        &CopcWriteMetadata::default(),
    )
    .unwrap();
    std::fs::read(&path).unwrap()
}

#[test]
fn open_fetches_metadata_but_no_point_chunks() {
    let bytes = synthetic_copc_bytes(5_000);
    let source = RecordingSource::new(bytes);
    let handle = source.clone();

    let reader = CopcRangeReader::open(source).unwrap();

    let point_data_start = u64::from(reader.header().offset_to_point_data);
    let hierarchy_start = reader.copc_info().root_hier_offset;
    for (offset, len) in handle.requests() {
        let end = offset + len;
        assert!(
            end <= point_data_start || offset >= hierarchy_start - 60,
            "open fetched point data range {offset}..{end}"
        );
    }
}

#[test]
fn full_read_coalesces_adjacent_chunks_into_one_request() {
    let bytes = synthetic_copc_bytes(5_000);
    let source = RecordingSource::new(bytes);
    let handle = source.clone();
    let mut reader = CopcRangeReader::open(source).unwrap();
    let chunk_count = reader.hierarchy_for(PointQuery::all()).unwrap().len();
    assert!(chunk_count > 1, "fixture should span multiple chunks");
    handle.clear_requests();

    let batch = reader
        .read_columns(PointQuery::all(), ColumnSelection::xyz())
        .unwrap();

    assert_eq!(5_000, batch.len());
    // Writer chunks are contiguous, so a full read needs one coalesced fetch.
    assert_eq!(1, handle.requests().len());
}

#[test]
fn bounded_query_fetches_fewer_bytes_than_full_read() {
    let bytes = synthetic_copc_bytes(5_000);
    let source = RecordingSource::new(bytes);
    let handle = source.clone();
    let mut reader = CopcRangeReader::open(source).unwrap();
    handle.clear_requests();

    let full_points = reader.read_points(PointQuery::all()).unwrap();
    let full_bytes: u64 = handle.requests().iter().map(|(_, len)| len).sum();
    handle.clear_requests();

    let corner = Bounds::new((0.0, 0.0, 0.0), (30.0, 30.0, 63.0));
    let bounded = reader
        .read_points(PointQuery::new(
            LodSelection::All,
            BoundsSelection::Within(corner),
        ))
        .unwrap();
    let bounded_bytes: u64 = handle.requests().iter().map(|(_, len)| len).sum();

    assert!(!bounded.is_empty());
    assert!(bounded.len() < full_points.len());
    assert!(
        bounded_bytes < full_bytes,
        "bounded query fetched {bounded_bytes} bytes, full read fetched {full_bytes}"
    );
    assert!(bounded
        .iter()
        .all(|point| corner.contains_xyz(point.x, point.y, point.z)));
}

#[test]
fn ranged_reader_matches_seek_reader() {
    let bytes = synthetic_copc_bytes(5_000);

    let mut seek_reader = CopcReader::open(std::io::Cursor::new(bytes.clone())).unwrap();
    let mut expected_points = seek_reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    let mut seek_reader = CopcReader::open(std::io::Cursor::new(bytes.clone())).unwrap();
    let expected_columns = seek_reader
        .read_columns(PointQuery::all(), ColumnSelection::all())
        .unwrap();

    let mut ranged = CopcRangeReader::open(RecordingSource::new(bytes)).unwrap();
    let mut points = ranged.read_points(PointQuery::all()).unwrap();
    let columns = ranged
        .read_columns(PointQuery::all(), ColumnSelection::all())
        .unwrap();

    let key = |point: &las::Point| (point.gps_time.unwrap().to_bits(), point.intensity);
    expected_points.sort_by_key(key);
    points.sort_by_key(key);
    assert_eq!(expected_points.len(), points.len());
    for (expected, actual) in expected_points.iter().zip(&points) {
        assert_eq!(expected.x, actual.x);
        assert_eq!(expected.y, actual.y);
        assert_eq!(expected.z, actual.z);
        assert_eq!(expected.gps_time, actual.gps_time);
        assert_eq!(expected.intensity, actual.intensity);
    }

    assert_eq!(expected_columns.len(), columns.len());
    assert_sorted_column_eq(&expected_columns, &columns, LasDimension::GpsTime);
    assert_sorted_column_eq(&expected_columns, &columns, LasDimension::X);
}

fn assert_sorted_column_eq(
    expected: &LasColumnBatch,
    actual: &LasColumnBatch,
    dimension: LasDimension,
) {
    let (Some(ColumnData::F64(expected)), Some(ColumnData::F64(actual))) =
        (expected.column(dimension), actual.column(dimension))
    else {
        panic!("{dimension:?} column missing or non-F64");
    };
    let mut expected = expected.clone();
    let mut actual = actual.clone();
    expected.sort_by(f64::total_cmp);
    actual.sort_by(f64::total_cmp);
    assert_eq!(expected, actual);
}

// --- lazy child-page loading over a crafted two-page hierarchy ---

#[test]
fn child_hierarchy_pages_load_lazily_per_query() {
    let (bytes, child_page_offset) = copc_with_child_hierarchy_page();
    let source = RecordingSource::new(bytes);
    let handle = source.clone();
    let mut reader = CopcRangeReader::open(source).unwrap();

    // A root-level query must not touch the child page.
    let root_entries = reader
        .hierarchy_for(PointQuery::new(
            LodSelection::Level(0),
            BoundsSelection::All,
        ))
        .unwrap();
    assert_eq!(1, root_entries.len());
    assert!(
        !handle
            .requests()
            .iter()
            .any(|(offset, _)| *offset == child_page_offset),
        "root-level query fetched the child hierarchy page"
    );

    // A deeper query loads it.
    let all_entries = reader.hierarchy_for(PointQuery::all()).unwrap();
    assert_eq!(3, all_entries.len());
    assert!(handle
        .requests()
        .iter()
        .any(|(offset, _)| *offset == child_page_offset));
}

/// Minimal COPC layout with a root page pointing at one child page; point
/// chunks are placeholders (hierarchy queries never decode them).
fn copc_with_child_hierarchy_page() -> (Vec<u8>, u64) {
    use laz::LazVlrBuilder;

    let mut laz_vlr_bytes = Vec::new();
    LazVlrBuilder::default()
        .with_point_format(6, 0)
        .unwrap()
        .with_variable_chunk_size()
        .build()
        .write_to(&mut laz_vlr_bytes)
        .unwrap();

    let offset_to_point_data = 375 + (54 + 160) + (54 + laz_vlr_bytes.len() as u32);
    let root_point_offset = u64::from(offset_to_point_data);
    let child_point_offset = root_point_offset + 100;
    let grandchild_point_offset = child_point_offset + 200;
    let evlr_start = grandchild_point_offset + 220;
    let root_hier_offset = evlr_start + 60;
    let root_hier_size = 2 * 32u64;
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
    write_vlr(&mut out, "copc", 1, &info.write_le_bytes());
    write_vlr(&mut out, "laszip encoded", 22204, &laz_vlr_bytes);
    assert_eq!(out.len(), offset_to_point_data as usize);
    out.resize(evlr_start as usize, 0);
    write_evlr_header(&mut out, "copc", 1000, root_page_bytes.len() as u64);
    assert_eq!(out.len() as u64, root_hier_offset);
    out.extend_from_slice(&root_page_bytes);
    assert_eq!(out.len() as u64, child_page_offset);
    out.extend_from_slice(&child_page_bytes);
    (out, child_page_offset)
}

fn write_las_header(out: &mut Vec<u8>, offset_to_point_data: u32, evlr_start: u64, points: u64) {
    out.resize(375, 0);
    out[0..4].copy_from_slice(b"LASF");
    out[24] = 1;
    out[25] = 4;
    out[94..96].copy_from_slice(&375u16.to_le_bytes());
    out[96..100].copy_from_slice(&offset_to_point_data.to_le_bytes());
    out[100..104].copy_from_slice(&2u32.to_le_bytes());
    out[104] = 6 | 0x80;
    out[105..107].copy_from_slice(&30u16.to_le_bytes());
    for (offset, value) in [(131, 0.001f64), (139, 0.001), (147, 0.001)] {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    for (offset, value) in [
        (179, 10.0f64),
        (187, -10.0),
        (195, 10.0),
        (203, -10.0),
        (211, 10.0),
        (219, -10.0),
    ] {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    out[235..243].copy_from_slice(&evlr_start.to_le_bytes());
    out[243..247].copy_from_slice(&1u32.to_le_bytes());
    out[247..255].copy_from_slice(&points.to_le_bytes());
}

fn write_vlr(out: &mut Vec<u8>, user_id: &str, record_id: u16, data: &[u8]) {
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&padded(user_id.as_bytes(), 16));
    out.extend_from_slice(&record_id.to_le_bytes());
    out.extend_from_slice(&(data.len() as u16).to_le_bytes());
    out.extend_from_slice(&padded(b"", 32));
    out.extend_from_slice(data);
}

fn write_evlr_header(out: &mut Vec<u8>, user_id: &str, record_id: u16, data_len: u64) {
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&padded(user_id.as_bytes(), 16));
    out.extend_from_slice(&record_id.to_le_bytes());
    out.extend_from_slice(&data_len.to_le_bytes());
    out.extend_from_slice(&padded(b"", 32));
}

fn padded(bytes: &[u8], len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    let count = bytes.len().min(len);
    out[..count].copy_from_slice(&bytes[..count]);
    out
}
