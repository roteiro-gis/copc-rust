use copc_core::Bounds;
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection};
use copc_writer::{
    write_source, CopcPointFields, CopcPointSource, CopcWriteMetadata, CopcWriterParams,
};

struct VecSource {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for VecSource {
    fn len(&self) -> usize {
        self.points.len()
    }

    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        let point = &self.points[index];
        (point.x, point.y, point.z)
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> copc_core::Result<()> {
        out.clone_from(&self.points[index]);
        Ok(())
    }
}

#[test]
fn reader_lod_resolution_bounds_and_size_hints_match_hierarchy() {
    let source = VecSource {
        points: grid_points(7_000),
    };
    let bounds = source_bounds(&source);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("query-conformance.copc.laz");

    write_source(
        &path,
        &source,
        false,
        bounds,
        &CopcWriterParams::new(96),
        &CopcWriteMetadata::default(),
    )
    .unwrap();

    let file = CopcFile::open(&path).unwrap();
    let entries = file.hierarchy_walk();
    assert!(
        entries.iter().any(|entry| entry.key.level >= 2),
        "fixture must subdivide deeply enough to exercise LOD ranges"
    );
    assert_eq!(
        entries
            .iter()
            .filter(|entry| entry.has_point_data())
            .map(|entry| entry.point_count as usize)
            .sum::<usize>(),
        source.len()
    );

    let expected_level_range = hierarchy_point_count(&file, 1..3);
    let mut reader = CopcReader::from_path(&path).unwrap();
    let mut iter = reader
        .points(LodSelection::LevelMinMax(1, 3), BoundsSelection::All)
        .unwrap();
    assert_eq!(
        iter.size_hint(),
        (expected_level_range, Some(expected_level_range))
    );
    let level_range_points = iter
        .by_ref()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert_eq!(level_range_points.len(), expected_level_range);

    let expected_resolution = hierarchy_point_count(&file, 0..2);
    let mut reader = CopcReader::from_path(&path).unwrap();
    let resolution_points = reader
        .points(
            LodSelection::Resolution(file.copc_info().spacing / 2.0),
            BoundsSelection::All,
        )
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert_eq!(resolution_points.len(), expected_resolution);

    let query_bounds = Bounds::new((-8.0, -8.0, -4.0), (9.0, 10.0, 5.0));
    let mut reader = CopcReader::from_path(&path).unwrap();
    let iter = reader
        .points(LodSelection::All, BoundsSelection::Within(query_bounds))
        .unwrap();
    let (lower, upper) = iter.size_hint();
    assert_eq!(lower, 0, "bounded reads are not exact-size upfront");
    let bounded_points = iter.collect::<copc_core::Result<Vec<_>>>().unwrap();
    assert!(!bounded_points.is_empty());
    assert!(bounded_points.len() < source.len());
    assert!(upper.is_some_and(|upper| upper >= bounded_points.len()));
    assert!(bounded_points
        .iter()
        .all(|point| query_bounds.contains_xyz(point.x, point.y, point.z)));

    assert!(CopcReader::from_path(&path)
        .unwrap()
        .points(LodSelection::Level(-1), BoundsSelection::All)
        .is_err());
    assert!(CopcReader::from_path(&path)
        .unwrap()
        .points(LodSelection::Resolution(0.0), BoundsSelection::All)
        .is_err());
}

fn grid_points(count: usize) -> Vec<CopcPointFields> {
    (0..count)
        .map(|i| {
            let x = (i % 31) as f64 - 15.0;
            let y = ((i / 31) % 29) as f64 - 14.0;
            let z = ((i / (31 * 29)) % 23) as f64 - 11.0;
            CopcPointFields {
                x,
                y,
                z,
                intensity: (i % u16::MAX as usize) as u16,
                return_number: ((i % 4) + 1) as u8,
                number_of_returns: 4,
                synthetic: u8::from(i % 11 == 0),
                key_point: u8::from(i % 13 == 0),
                withheld: u8::from(i % 17 == 0),
                overlap: u8::from(i % 19 == 0),
                scan_channel: (i % 4) as u8,
                scan_direction_flag: (i % 2) as u8,
                edge_of_flight_line: u8::from(i % 7 == 0),
                classification: (i % 32) as u8,
                user_data: (i % 251) as u8,
                scan_angle: (i as f32 % 180.0) - 90.0,
                point_source_id: (i % u16::MAX as usize) as u16,
                gps_time: 1.0e9 + i as f64,
                red: 0,
                green: 0,
                blue: 0,
                extra_bytes: Vec::new(),
            }
        })
        .collect()
}

fn source_bounds(source: &VecSource) -> Bounds {
    source.points.iter().fold(
        Bounds::point(source.points[0].x, source.points[0].y, source.points[0].z),
        |mut bounds, point| {
            bounds.extend(point.x, point.y, point.z);
            bounds
        },
    )
}

fn hierarchy_point_count(file: &CopcFile, levels: std::ops::Range<i32>) -> usize {
    file.hierarchy_walk()
        .iter()
        .filter(|entry| entry.has_point_data() && levels.contains(&entry.key.level))
        .map(|entry| entry.point_count as usize)
        .sum()
}
