use std::path::{Path, PathBuf};

use copc_core::{Bounds, ColumnData, ColumnSelection, LasColumnBatch, LasDimension};
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection, PointQuery};
use las::point::ScanDirection;

#[test]
fn checked_in_copc_fixtures_open_iterate_and_filter() {
    let fixtures = fixture_paths();
    assert_fixture_set("pdal", &fixtures);
    assert_fixture_set("qgis", &fixtures);

    for path in fixtures {
        assert_fixture(&path);
    }
}

fn assert_fixture_set(source: &str, fixtures: &[PathBuf]) {
    let source_dir = fixtures_dir().join(source);
    assert!(
        fixtures.iter().any(|path| path.starts_with(&source_dir)),
        "missing checked-in {source} fixture under {}",
        source_dir.display()
    );
}

fn assert_fixture(path: &Path) {
    let file = CopcFile::open(path).unwrap_or_else(|err| panic!("open {}: {err}", path.display()));
    let header = file.header();
    assert!(
        header.number_of_points() > 0,
        "{} should contain points",
        path.display()
    );
    assert!(
        file.hierarchy_entries().any(|entry| entry.has_point_data()),
        "{} should expose point-data hierarchy entries",
        path.display()
    );
    assert_eq!(
        file.hierarchy_entries()
            .filter(|entry| entry.has_point_data())
            .map(|entry| entry.point_count as u64)
            .sum::<u64>(),
        header.number_of_points(),
        "{} hierarchy point counts must match LAS header",
        path.display()
    );

    let mut reader = CopcReader::from_path(path).unwrap();
    let all_points = reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert_eq!(all_points.len() as u64, header.number_of_points());

    let mut reader = CopcReader::from_path(path).unwrap();
    let columns = reader
        .read_columns(PointQuery::all(), ColumnSelection::all())
        .unwrap();
    assert_columns_match_points(&columns, &all_points);

    let full_bounds = Bounds::new(
        (header.min_x, header.min_y, header.min_z),
        (header.max_x, header.max_y, header.max_z),
    );
    let mut reader = CopcReader::from_path(path).unwrap();
    let bounded_points = reader
        .points(LodSelection::All, BoundsSelection::Within(full_bounds))
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    let outside_full_bounds = all_points
        .iter()
        .filter(|point| !full_bounds.contains_xyz(point.x, point.y, point.z))
        .take(4)
        .map(|point| (point.x, point.y, point.z))
        .collect::<Vec<_>>();
    assert_eq!(
        bounded_points.len(),
        all_points.len(),
        "{} full-file bounds selection should keep every point; first points outside header bounds: {:?}",
        path.display(),
        outside_full_bounds
    );

    let mut reader = CopcReader::from_path(path).unwrap();
    let bounded_intensity = reader
        .read_columns(
            PointQuery::new(LodSelection::All, BoundsSelection::Within(full_bounds)),
            ColumnSelection::from_dimensions([LasDimension::Intensity]),
        )
        .unwrap();
    assert_eq!(bounded_intensity.len(), all_points.len());
    assert!(bounded_intensity.column(LasDimension::X).is_none());
    assert_eq!(
        column_u16(&bounded_intensity, LasDimension::Intensity),
        all_points
            .iter()
            .map(|point| point.intensity)
            .collect::<Vec<_>>()
            .as_slice()
    );

    let mut reader = CopcReader::from_path(path).unwrap();
    let root_points = reader
        .points(LodSelection::Level(0), BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert!(
        !root_points.is_empty(),
        "{} should expose root LOD points",
        path.display()
    );
    assert!(root_points.len() <= all_points.len());
}

fn assert_columns_match_points(batch: &LasColumnBatch, points: &[las::Point]) {
    assert_eq!(batch.len(), points.len());
    assert_eq!(
        column_f64(batch, LasDimension::X),
        points
            .iter()
            .map(|point| point.x)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f64(batch, LasDimension::Y),
        points
            .iter()
            .map(|point| point.y)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f64(batch, LasDimension::Z),
        points
            .iter()
            .map(|point| point.z)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(batch, LasDimension::Intensity),
        points
            .iter()
            .map(|point| point.intensity)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(batch, LasDimension::ReturnNumber),
        points
            .iter()
            .map(|point| point.return_number)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(batch, LasDimension::NumberOfReturns),
        points
            .iter()
            .map(|point| point.number_of_returns)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(batch, LasDimension::Classification),
        points
            .iter()
            .map(|point| u8::from(point.classification))
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::ScanDirectionFlag),
        points
            .iter()
            .map(|point| point.scan_direction == ScanDirection::LeftToRight)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::EdgeOfFlightLine),
        points
            .iter()
            .map(|point| point.is_edge_of_flight_line)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f32(batch, LasDimension::ScanAngle),
        points
            .iter()
            .map(|point| point.scan_angle)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(batch, LasDimension::UserData),
        points
            .iter()
            .map(|point| point.user_data)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(batch, LasDimension::PointSourceId),
        points
            .iter()
            .map(|point| point.point_source_id)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::Synthetic),
        points
            .iter()
            .map(|point| point.is_synthetic)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::KeyPoint),
        points
            .iter()
            .map(|point| point.is_key_point)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::Withheld),
        points
            .iter()
            .map(|point| point.is_withheld)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_bool(batch, LasDimension::Overlap),
        points
            .iter()
            .map(|point| point.is_overlap)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(batch, LasDimension::ScanChannel),
        points
            .iter()
            .map(|point| point.scanner_channel)
            .collect::<Vec<_>>()
            .as_slice()
    );

    if batch.column(LasDimension::GpsTime).is_some() {
        assert_eq!(
            column_f64(batch, LasDimension::GpsTime),
            points
                .iter()
                .map(|point| point.gps_time.unwrap_or(0.0))
                .collect::<Vec<_>>()
                .as_slice()
        );
    }
    if batch.column(LasDimension::Red).is_some() {
        assert_eq!(
            column_u16(batch, LasDimension::Red),
            points
                .iter()
                .map(|point| point.color.unwrap_or_default().red)
                .collect::<Vec<_>>()
                .as_slice()
        );
        assert_eq!(
            column_u16(batch, LasDimension::Green),
            points
                .iter()
                .map(|point| point.color.unwrap_or_default().green)
                .collect::<Vec<_>>()
                .as_slice()
        );
        assert_eq!(
            column_u16(batch, LasDimension::Blue),
            points
                .iter()
                .map(|point| point.color.unwrap_or_default().blue)
                .collect::<Vec<_>>()
                .as_slice()
        );
    }
    if batch.column(LasDimension::Nir).is_some() {
        assert_eq!(
            column_u16(batch, LasDimension::Nir),
            points
                .iter()
                .map(|point| point.nir.unwrap_or(0))
                .collect::<Vec<_>>()
                .as_slice()
        );
    }
}

fn column_f64(batch: &LasColumnBatch, dimension: LasDimension) -> &[f64] {
    match batch.column(dimension).unwrap() {
        ColumnData::F64(values) => values,
        other => panic!(
            "{dimension:?} column has unexpected type {:?}",
            other.scalar()
        ),
    }
}

fn column_f32(batch: &LasColumnBatch, dimension: LasDimension) -> &[f32] {
    match batch.column(dimension).unwrap() {
        ColumnData::F32(values) => values,
        other => panic!(
            "{dimension:?} column has unexpected type {:?}",
            other.scalar()
        ),
    }
}

fn column_u16(batch: &LasColumnBatch, dimension: LasDimension) -> &[u16] {
    match batch.column(dimension).unwrap() {
        ColumnData::U16(values) => values,
        other => panic!(
            "{dimension:?} column has unexpected type {:?}",
            other.scalar()
        ),
    }
}

fn column_u8(batch: &LasColumnBatch, dimension: LasDimension) -> &[u8] {
    match batch.column(dimension).unwrap() {
        ColumnData::U8(values) => values,
        other => panic!(
            "{dimension:?} column has unexpected type {:?}",
            other.scalar()
        ),
    }
}

fn column_bool(batch: &LasColumnBatch, dimension: LasDimension) -> &[bool] {
    match batch.column(dimension).unwrap() {
        ColumnData::Bool(values) => values,
        other => panic!(
            "{dimension:?} column has unexpected type {:?}",
            other.scalar()
        ),
    }
}

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/external")
}

fn fixture_paths() -> Vec<PathBuf> {
    let mut paths = Vec::new();
    collect_fixtures(&fixtures_dir(), &mut paths);
    paths.sort();
    paths
}

fn collect_fixtures(dir: &Path, paths: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };

    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect_fixtures(&path, paths);
        } else if path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.ends_with(".copc.laz"))
        {
            paths.push(path);
        }
    }
}
