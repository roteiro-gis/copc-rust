use copc_core::{Bounds, ColumnData, LasColumnBatch, LasDimension};
use copc_reader::{BoundsSelection, ColumnSelection, CopcReader, LodSelection, PointQuery};
use copc_writer::{write_source, CopcPointFields, CopcPointSource, CopcWriterParams};

struct VecSource {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for VecSource {
    fn len(&self) -> usize {
        self.points.len()
    }

    fn xyz(&self, index: usize) -> (f64, f64, f64) {
        let point = self.points[index];
        (point.x, point.y, point.z)
    }

    fn fields(&self, index: usize) -> copc_core::Result<CopcPointFields> {
        Ok(self.points[index])
    }
}

#[test]
fn read_columns_matches_synthetic_copc_rows() {
    let source = VecSource {
        points: grid_points(1_500),
    };
    let bounds = source_bounds(&source.points);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("columns.copc.laz");

    write_source(
        &path,
        &source,
        true,
        bounds,
        &CopcWriterParams {
            max_points_per_node: 64,
            max_depth: 6,
        },
    )
    .unwrap();

    let all_query = PointQuery::all();
    let all_points = read_rows(&path, all_query);
    let all_columns = read_columns(&path, all_query, ColumnSelection::all());

    assert_eq!(all_columns.len(), all_points.len());
    assert_eq!(
        column_f64(&all_columns, LasDimension::X),
        all_points
            .iter()
            .map(|point| point.x)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f64(&all_columns, LasDimension::Y),
        all_points
            .iter()
            .map(|point| point.y)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f64(&all_columns, LasDimension::Z),
        all_points
            .iter()
            .map(|point| point.z)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(&all_columns, LasDimension::Classification),
        all_points
            .iter()
            .map(|point| u8::from(point.classification))
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(&all_columns, LasDimension::Intensity),
        all_points
            .iter()
            .map(|point| point.intensity)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u8(&all_columns, LasDimension::ReturnNumber),
        all_points
            .iter()
            .map(|point| point.return_number)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_f64(&all_columns, LasDimension::GpsTime),
        all_points
            .iter()
            .map(|point| point.gps_time.unwrap_or(0.0))
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(&all_columns, LasDimension::Red),
        all_points
            .iter()
            .map(|point| point.color.unwrap_or_default().red)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(&all_columns, LasDimension::Green),
        all_points
            .iter()
            .map(|point| point.color.unwrap_or_default().green)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert_eq!(
        column_u16(&all_columns, LasDimension::Blue),
        all_points
            .iter()
            .map(|point| point.color.unwrap_or_default().blue)
            .collect::<Vec<_>>()
            .as_slice()
    );

    let selected = read_columns(
        &path,
        all_query,
        ColumnSelection::from_dimensions([
            LasDimension::X,
            LasDimension::Y,
            LasDimension::Z,
            LasDimension::Classification,
        ]),
    );
    assert_eq!(selected.len(), all_points.len());
    assert_eq!(
        selected
            .columns
            .iter()
            .map(|(spec, _)| spec.dimension)
            .collect::<Vec<_>>(),
        vec![
            LasDimension::X,
            LasDimension::Y,
            LasDimension::Z,
            LasDimension::Classification,
        ]
    );

    let query_bounds = Bounds::new((-4.0, -3.5, -12.0), (5.0, 4.5, -9.0));
    let bounded_query = PointQuery::new(LodSelection::All, BoundsSelection::Within(query_bounds));
    let bounded_points = read_rows(&path, bounded_query);
    let bounded_columns = read_columns(&path, bounded_query, ColumnSelection::all());
    assert!(!bounded_points.is_empty());
    assert!(bounded_points.len() < all_points.len());
    assert_eq!(bounded_columns.len(), bounded_points.len());
    assert_eq!(
        column_u16(&bounded_columns, LasDimension::Intensity),
        bounded_points
            .iter()
            .map(|point| point.intensity)
            .collect::<Vec<_>>()
            .as_slice()
    );

    let lod_query = PointQuery::new(LodSelection::Level(1), BoundsSelection::All);
    let lod_points = read_rows(&path, lod_query);
    let lod_columns = read_columns(&path, lod_query, ColumnSelection::all());
    assert!(!lod_points.is_empty());
    assert!(lod_points.len() < all_points.len());
    assert_eq!(lod_columns.len(), lod_points.len());
}

fn read_rows(path: &std::path::Path, query: PointQuery) -> Vec<las::Point> {
    let mut reader = CopcReader::from_path(path).unwrap();
    reader
        .points_for_query(query)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap()
}

fn read_columns(
    path: &std::path::Path,
    query: PointQuery,
    selection: ColumnSelection,
) -> LasColumnBatch {
    let mut reader = CopcReader::from_path(path).unwrap();
    reader.read_columns(query, selection).unwrap()
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
                intensity: (10_000 + i) as u16,
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
                scan_angle: (i as f32 % 90.0) - 45.0,
                point_source_id: (i % u16::MAX as usize) as u16,
                gps_time: 1.0e9 + i as f64 * 0.25,
                red: (i % u16::MAX as usize) as u16,
                green: ((i * 3) % u16::MAX as usize) as u16,
                blue: ((i * 7) % u16::MAX as usize) as u16,
            }
        })
        .collect()
}

fn source_bounds(points: &[CopcPointFields]) -> Bounds {
    points.iter().fold(
        Bounds::point(points[0].x, points[0].y, points[0].z),
        |mut bounds, point| {
            bounds.extend(point.x, point.y, point.z);
            bounds
        },
    )
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
