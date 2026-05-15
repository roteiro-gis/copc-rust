use copc_core::Bounds;
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection};
use copc_writer::{write_source, CopcPointFields, CopcPointSource, CopcWriterParams};

struct VecSource {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for VecSource {
    fn len(&self) -> usize {
        self.points.len()
    }

    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        let p = self.points[index];
        (p.x, p.y, p.z)
    }

    fn fields(&self, index: usize) -> copc_core::Result<CopcPointFields> {
        Ok(self.points[index])
    }
}

#[test]
fn writer_output_parses_with_reader_hierarchy() {
    let mut points = Vec::new();
    for i in 0..1_000 {
        let t = i as f64 / 1_000.0 * std::f64::consts::TAU;
        points.push(CopcPointFields {
            x: 100.0 + 50.0 * t.cos(),
            y: 200.0 + 50.0 * t.sin(),
            z: 10.0 + i as f64 * 0.01,
            intensity: i as u16,
            return_number: 1,
            number_of_returns: 1,
            synthetic: 0,
            key_point: 0,
            withheld: 0,
            overlap: 0,
            scan_channel: 0,
            scan_direction_flag: 0,
            edge_of_flight_line: 0,
            classification: 2,
            user_data: 0,
            scan_angle_rank: 0,
            point_source_id: 1,
            gps_time: 1.0e9 + i as f64,
            red: 0,
            green: 0,
            blue: 0,
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
    let path = dir.path().join("synthetic.copc.laz");
    write_source(
        &path,
        &source,
        false,
        bounds,
        &CopcWriterParams {
            max_points_per_node: 128,
            max_depth: 6,
        },
    )
    .unwrap();

    let file = CopcFile::open(&path).unwrap();
    assert_eq!(file.header().number_of_points, source.len() as u64);
    assert_eq!(file.header().point_data_record_format & 0x7F, 6);
    assert!(file.copc_info().root_hier_size > 0);
    let entries = file.hierarchy_walk();
    assert!(entries.len() > 1);
    assert_eq!(
        entries
            .iter()
            .map(|entry| entry.point_count as usize)
            .sum::<usize>(),
        source.len()
    );

    let mut reader = CopcReader::open(std::fs::File::open(&path).unwrap()).unwrap();
    let all_points = reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert_eq!(all_points.len(), source.len());
    assert!(all_points
        .iter()
        .any(|point| u8::from(point.classification) == 2));

    let mut reader = CopcReader::open(std::fs::File::open(&path).unwrap()).unwrap();
    let root_points = reader
        .points(LodSelection::Level(0), BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert!(!root_points.is_empty());
    assert!(root_points.len() < source.len());

    let query_bounds = Bounds::new((125.0, 150.0, 0.0), (151.0, 251.0, 100.0));
    let mut reader = CopcReader::open(std::fs::File::open(&path).unwrap()).unwrap();
    let bounded_points = reader
        .points(LodSelection::All, BoundsSelection::Within(query_bounds))
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert!(!bounded_points.is_empty());
    assert!(bounded_points.len() < all_points.len());
    assert!(bounded_points
        .iter()
        .all(|point| query_bounds.contains_xyz(point.x, point.y, point.z)));
}
