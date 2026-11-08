use copc_core::{Bounds, ColumnData, ColumnSpec, LasColumnBatch, LasDimension};
use copc_reader::{BoundsSelection, ColumnSelection, CopcReader, LodSelection, PointQuery};
use copc_writer::{
    write_source, ColumnBatchSource, CopcPointFields, CopcPointSource, CopcWriteMetadata,
    CopcWriterParams,
};

struct VecSource {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for VecSource {
    fn len(&self) -> usize {
        self.points.len()
    }

    fn xyz(&self, index: usize) -> copc_core::Result<(f64, f64, f64)> {
        let point = &self.points[index];
        Ok((point.x, point.y, point.z))
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> copc_core::Result<()> {
        out.clone_from(&self.points[index]);
        Ok(())
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
        &CopcWriterParams::new(64),
        &CopcWriteMetadata::default(),
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
    // `las::Point` folds legacy class 12 into 1, so check classification
    // against the source values, keyed by each row's unique GPS time.
    assert_eq!(
        column_u8(&all_columns, LasDimension::Classification),
        all_points
            .iter()
            .map(|point| source.points[grid_index(point)].classification)
            .collect::<Vec<_>>()
            .as_slice()
    );
    assert!(column_u8(&all_columns, LasDimension::Classification).contains(&12));
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

#[test]
fn column_batch_source_writes_columns_readable_by_reader() {
    let batch = column_batch(384);
    let source = ColumnBatchSource::new(&batch).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("column-batch-source.copc.laz");

    write_source(
        &path,
        &source,
        source.has_color(),
        source.bounds().unwrap(),
        &CopcWriterParams::new(1_024),
        &CopcWriteMetadata::default(),
    )
    .unwrap();

    let roundtripped = read_columns(&path, PointQuery::all(), ColumnSelection::all());
    assert_eq!(roundtripped.len(), batch.len());
    for (spec, expected) in &batch.columns {
        let actual = roundtripped
            .column(spec.dimension)
            .unwrap_or_else(|| panic!("missing {:?} column", spec.dimension));
        assert_column_data_eq(spec.dimension, expected, actual);
    }
}

/// Column reads decode only the LAZ layers their dimensions need; every
/// single-dimension read must still match the full-record row decoder.
#[test]
fn single_dimension_reads_match_rows() {
    struct ExtraBytesSource(VecSource);

    impl CopcPointSource for ExtraBytesSource {
        fn len(&self) -> usize {
            self.0.len()
        }

        fn xyz(&self, index: usize) -> copc_core::Result<(f64, f64, f64)> {
            self.0.xyz(index)
        }

        fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> copc_core::Result<()> {
            self.0.fields_into(index, out)?;
            out.extra_bytes = vec![index as u8, (index >> 8) as u8, 0xA5];
            Ok(())
        }

        fn extra_byte_count(&self) -> u16 {
            3
        }
    }

    let source = ExtraBytesSource(VecSource {
        points: grid_points(1_000),
    });
    let bounds = source_bounds(&source.0.points);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("selective.copc.laz");
    write_source(
        &path,
        &source,
        true,
        bounds,
        &CopcWriterParams::new(128),
        &CopcWriteMetadata::default(),
    )
    .unwrap();

    let sub_bounds = Bounds::new((-8.0, -9.0, -11.5), (9.0, 7.0, -9.5));
    for query in [
        PointQuery::all(),
        PointQuery::new(LodSelection::All, BoundsSelection::Within(sub_bounds)),
    ] {
        // Rows come from the full-record decoder, independent of column
        // layer selection.
        let rows = read_rows(&path, query);
        assert!(!rows.is_empty());
        let dimensions = read_columns(&path, query, ColumnSelection::all())
            .columns
            .into_iter()
            .map(|(spec, _)| spec.dimension);
        for dimension in dimensions {
            let single = read_columns(&path, query, ColumnSelection::from_dimensions([dimension]));
            assert_eq!(rows.len(), single.len(), "{dimension:?}");
            assert_eq!(
                Some(&expected_column(dimension, &rows, &source.0.points)),
                single.column(dimension),
                "{dimension:?} single-dimension read differs from rows for {query:?}"
            );
        }
    }
}

/// Index of the `grid_points` source point a row came from, recovered from
/// its unique GPS time.
fn grid_index(point: &las::Point) -> usize {
    ((point.gps_time.unwrap() - 1.0e9) / 0.25).round() as usize
}

fn expected_column(
    dimension: LasDimension,
    rows: &[las::Point],
    source: &[CopcPointFields],
) -> ColumnData {
    let f64s = |value: fn(&las::Point) -> f64| ColumnData::F64(rows.iter().map(value).collect());
    let u16s = |value: fn(&las::Point) -> u16| ColumnData::U16(rows.iter().map(value).collect());
    let u8s = |value: fn(&las::Point) -> u8| ColumnData::U8(rows.iter().map(value).collect());
    let bools = |value: fn(&las::Point) -> bool| ColumnData::Bool(rows.iter().map(value).collect());
    match dimension {
        LasDimension::X => f64s(|p| p.x),
        LasDimension::Y => f64s(|p| p.y),
        LasDimension::Z => f64s(|p| p.z),
        LasDimension::Intensity => u16s(|p| p.intensity),
        LasDimension::ReturnNumber => u8s(|p| p.return_number),
        LasDimension::NumberOfReturns => u8s(|p| p.number_of_returns),
        // `las::Point` folds legacy class 12 into 1; columns report the
        // stored byte.
        LasDimension::Classification => ColumnData::U8(
            rows.iter()
                .map(|p| source[grid_index(p)].classification)
                .collect(),
        ),
        LasDimension::ScanDirectionFlag => {
            bools(|p| matches!(p.scan_direction, las::point::ScanDirection::LeftToRight))
        }
        LasDimension::EdgeOfFlightLine => bools(|p| p.is_edge_of_flight_line),
        LasDimension::ScanAngle => ColumnData::F32(rows.iter().map(|p| p.scan_angle).collect()),
        LasDimension::UserData => u8s(|p| p.user_data),
        LasDimension::PointSourceId => u16s(|p| p.point_source_id),
        LasDimension::Synthetic => bools(|p| p.is_synthetic),
        LasDimension::KeyPoint => bools(|p| p.is_key_point),
        LasDimension::Withheld => bools(|p| p.is_withheld),
        LasDimension::Overlap => bools(|p| p.is_overlap),
        LasDimension::ScanChannel => u8s(|p| p.scanner_channel),
        LasDimension::GpsTime => f64s(|p| p.gps_time.unwrap()),
        LasDimension::Red => u16s(|p| p.color.unwrap().red),
        LasDimension::Green => u16s(|p| p.color.unwrap().green),
        LasDimension::Blue => u16s(|p| p.color.unwrap().blue),
        LasDimension::ExtraBytes => {
            ColumnData::U8(rows.iter().flat_map(|p| p.extra_bytes.clone()).collect())
        }
        other => panic!("no row reference for {other:?}"),
    }
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
                synthetic: u8::from(i.is_multiple_of(11)),
                key_point: u8::from(i.is_multiple_of(13)),
                withheld: u8::from(i.is_multiple_of(17)),
                overlap: u8::from(i.is_multiple_of(19)),
                scan_channel: (i % 4) as u8,
                scan_direction_flag: (i % 2) as u8,
                edge_of_flight_line: u8::from(i.is_multiple_of(7)),
                classification: (i % 32) as u8,
                user_data: (i % 251) as u8,
                scan_angle: (i as f32 % 90.0) - 45.0,
                point_source_id: (i % u16::MAX as usize) as u16,
                gps_time: 1.0e9 + i as f64 * 0.25,
                red: (i % u16::MAX as usize) as u16,
                green: ((i * 3) % u16::MAX as usize) as u16,
                blue: ((i * 7) % u16::MAX as usize) as u16,
                nir: 0,
                extra_bytes: Vec::new(),
            }
        })
        .collect()
}

fn column_batch(count: usize) -> LasColumnBatch {
    let mut x = Vec::with_capacity(count);
    let mut y = Vec::with_capacity(count);
    let mut z = Vec::with_capacity(count);
    let mut intensity = Vec::with_capacity(count);
    let mut return_number = Vec::with_capacity(count);
    let mut number_of_returns = Vec::with_capacity(count);
    let mut classification = Vec::with_capacity(count);
    let mut scan_direction_flag = Vec::with_capacity(count);
    let mut edge_of_flight_line = Vec::with_capacity(count);
    let mut scan_angle = Vec::with_capacity(count);
    let mut user_data = Vec::with_capacity(count);
    let mut point_source_id = Vec::with_capacity(count);
    let mut synthetic = Vec::with_capacity(count);
    let mut key_point = Vec::with_capacity(count);
    let mut withheld = Vec::with_capacity(count);
    let mut overlap = Vec::with_capacity(count);
    let mut scan_channel = Vec::with_capacity(count);
    let mut gps_time = Vec::with_capacity(count);
    let mut red = Vec::with_capacity(count);
    let mut green = Vec::with_capacity(count);
    let mut blue = Vec::with_capacity(count);

    for i in 0..count {
        x.push((i % 24) as f64 * 0.5 - 6.0);
        y.push(((i / 24) % 16) as f64 * 0.25 - 2.0);
        z.push((i / (24 * 16)) as f64 * 0.125 + 10.0);
        intensity.push((20_000 + i) as u16);
        return_number.push(((i % 4) + 1) as u8);
        number_of_returns.push(4);
        classification.push(if i.is_multiple_of(2) { 2 } else { 6 });
        scan_direction_flag.push(i.is_multiple_of(2));
        edge_of_flight_line.push(i.is_multiple_of(5));
        scan_angle.push(((i % 181) as f32 - 90.0) * 0.006);
        user_data.push((i % 251) as u8);
        point_source_id.push((1_000 + i) as u16);
        synthetic.push(i.is_multiple_of(7));
        key_point.push(i.is_multiple_of(11));
        withheld.push(i.is_multiple_of(13));
        overlap.push(i.is_multiple_of(17));
        scan_channel.push((i % 4) as u8);
        gps_time.push(1.0e9 + i as f64 * 0.25);
        red.push((i * 3) as u16);
        green.push((i * 5) as u16);
        blue.push((i * 7) as u16);
    }

    LasColumnBatch::new(vec![
        column(LasDimension::X, ColumnData::F64(x)),
        column(LasDimension::Y, ColumnData::F64(y)),
        column(LasDimension::Z, ColumnData::F64(z)),
        column(LasDimension::Intensity, ColumnData::U16(intensity)),
        column(LasDimension::ReturnNumber, ColumnData::U8(return_number)),
        column(
            LasDimension::NumberOfReturns,
            ColumnData::U8(number_of_returns),
        ),
        column(LasDimension::Classification, ColumnData::U8(classification)),
        column(
            LasDimension::ScanDirectionFlag,
            ColumnData::Bool(scan_direction_flag),
        ),
        column(
            LasDimension::EdgeOfFlightLine,
            ColumnData::Bool(edge_of_flight_line),
        ),
        column(LasDimension::ScanAngle, ColumnData::F32(scan_angle)),
        column(LasDimension::UserData, ColumnData::U8(user_data)),
        column(
            LasDimension::PointSourceId,
            ColumnData::U16(point_source_id),
        ),
        column(LasDimension::Synthetic, ColumnData::Bool(synthetic)),
        column(LasDimension::KeyPoint, ColumnData::Bool(key_point)),
        column(LasDimension::Withheld, ColumnData::Bool(withheld)),
        column(LasDimension::Overlap, ColumnData::Bool(overlap)),
        column(LasDimension::ScanChannel, ColumnData::U8(scan_channel)),
        column(LasDimension::GpsTime, ColumnData::F64(gps_time)),
        column(LasDimension::Red, ColumnData::U16(red)),
        column(LasDimension::Green, ColumnData::U16(green)),
        column(LasDimension::Blue, ColumnData::U16(blue)),
    ])
    .unwrap()
}

fn column(dimension: LasDimension, data: ColumnData) -> (ColumnSpec, ColumnData) {
    (
        ColumnSpec::default_for(dimension).expect("fixed LAS column spec"),
        data,
    )
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

fn assert_column_data_eq(dimension: LasDimension, expected: &ColumnData, actual: &ColumnData) {
    match (expected, actual) {
        (ColumnData::F64(expected), ColumnData::F64(actual)) => {
            assert_eq!(expected.len(), actual.len(), "{dimension:?} length");
            for (index, (&expected, &actual)) in expected.iter().zip(actual).enumerate() {
                assert!(
                    (expected - actual).abs() <= 1e-9,
                    "{dimension:?} differs at row {index}: expected {expected}, got {actual}"
                );
            }
        }
        (ColumnData::F32(expected), ColumnData::F32(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::I64(expected), ColumnData::I64(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::I32(expected), ColumnData::I32(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::I16(expected), ColumnData::I16(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::I8(expected), ColumnData::I8(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::U64(expected), ColumnData::U64(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::U32(expected), ColumnData::U32(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::U16(expected), ColumnData::U16(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::U8(expected), ColumnData::U8(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        (ColumnData::Bool(expected), ColumnData::Bool(actual)) => {
            assert_eq!(expected, actual, "{dimension:?}");
        }
        _ => panic!(
            "{dimension:?} scalar mismatch: expected {:?}, got {:?}",
            expected.scalar(),
            actual.scalar()
        ),
    }
}

#[test]
fn column_batch_source_writes_nir_as_point_format_8() {
    let mut batch = column_batch(384);
    let nir: Vec<u16> = (0..384u16).map(|i| 50_000 + i).collect();
    batch
        .columns
        .push(column(LasDimension::Nir, ColumnData::U16(nir.clone())));
    let source = ColumnBatchSource::new(&batch).unwrap();
    assert!(source.has_nir());
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("nir.copc.laz");

    write_source(
        &path,
        &source,
        source.has_color(),
        source.bounds().unwrap(),
        &CopcWriterParams::new(1_024),
        &CopcWriteMetadata::default(),
    )
    .unwrap();

    let mut reader = CopcReader::from_path(&path).unwrap();
    assert_eq!(8, reader.header().point_data_record_format & 0x3F);
    let roundtripped = reader
        .read_columns(PointQuery::all(), ColumnSelection::all())
        .unwrap();
    for (spec, expected) in &batch.columns {
        let actual = roundtripped
            .column(spec.dimension)
            .unwrap_or_else(|| panic!("missing {:?} column", spec.dimension));
        assert_column_data_eq(spec.dimension, expected, actual);
    }

    let mut without_rgb = column_batch(4);
    without_rgb.columns.retain(|(spec, _)| {
        !matches!(
            spec.dimension,
            LasDimension::Red | LasDimension::Green | LasDimension::Blue
        )
    });
    without_rgb
        .columns
        .push(column(LasDimension::Nir, ColumnData::U16(vec![1; 4])));
    let err = ColumnBatchSource::new(&without_rgb).err().unwrap();
    assert!(err.to_string().contains("requires Red, Green, and Blue"));
}
