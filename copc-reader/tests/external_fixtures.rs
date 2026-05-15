use std::path::{Path, PathBuf};

use copc_core::Bounds;
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection};

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
