use byteorder::{LittleEndian, ReadBytesExt};
use copc_core::{Bounds, NeverCancel, StreamingLayout};
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection};
use copc_writer::{
    convert_las_to_copc_streaming, write_source, write_streaming_with_cancel, CopcPointFields,
    CopcPointSource, CopcWriterParams,
};
use las::point::ScanDirection;
use las::{Color, Point};
use las::{Read as _, Write as _};
use std::io::{Read as _, Seek as _, SeekFrom};

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

#[test]
fn writer_round_trips_fields_through_copc_and_las_readers() {
    let points = vec![
        CopcPointFields {
            x: 10.125,
            y: -20.5,
            z: 3.25,
            intensity: 123,
            return_number: 1,
            number_of_returns: 3,
            synthetic: 1,
            key_point: 0,
            withheld: 0,
            overlap: 1,
            scan_channel: 2,
            scan_direction_flag: 1,
            edge_of_flight_line: 0,
            classification: 7,
            user_data: 42,
            scan_angle_rank: -30,
            point_source_id: 77,
            gps_time: 12345.5,
            red: 1000,
            green: 2000,
            blue: 3000,
        },
        CopcPointFields {
            x: 11.0,
            y: -19.25,
            z: 4.75,
            intensity: 456,
            return_number: 2,
            number_of_returns: 3,
            synthetic: 0,
            key_point: 1,
            withheld: 1,
            overlap: 0,
            scan_channel: 1,
            scan_direction_flag: 0,
            edge_of_flight_line: 1,
            classification: 9,
            user_data: 7,
            scan_angle_rank: 15,
            point_source_id: 78,
            gps_time: 12346.25,
            red: 4000,
            green: 5000,
            blue: 6000,
        },
    ];
    let bounds = source_bounds(&points);
    let source = VecSource {
        points: points.clone(),
    };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("fields.copc.laz");

    write_source(&path, &source, true, bounds, &CopcWriterParams::default()).unwrap();

    let mut copc_reader = CopcReader::open(std::fs::File::open(&path).unwrap()).unwrap();
    let copc_points = copc_reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();
    assert_las_points_match_fields(&points, &copc_points);

    let mut las_reader = las::Reader::from_path(&path).unwrap();
    assert_eq!(7, las_reader.header().point_format().to_u8().unwrap());
    assert_eq!(points.len() as u64, las_reader.header().number_of_points());
    let las_points = las_reader
        .points()
        .collect::<las::Result<Vec<_>>>()
        .unwrap();
    assert_las_points_match_fields(&points, &las_points);
}

#[test]
fn writer_emits_gps_time_range_and_return_histogram() {
    let specs = [
        (0.0, 100.0, 1u8),
        (1.0, 75.0, 2u8),
        (2.0, 120.0, 2u8),
        (3.0, 90.0, 15u8),
    ];
    let points: Vec<CopcPointFields> = specs
        .iter()
        .map(|&(x, gps_time, return_number)| {
            let mut point = point_fields(x, x, x);
            point.gps_time = gps_time;
            point.return_number = return_number;
            point.number_of_returns = 15;
            point
        })
        .collect();
    let bounds = points.iter().fold(
        Bounds::point(points[0].x, points[0].y, points[0].z),
        |mut bounds, point| {
            bounds.extend(point.x, point.y, point.z);
            bounds
        },
    );
    let source = VecSource { points };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("point-stats.copc.laz");

    write_source(&path, &source, false, bounds, &CopcWriterParams::default()).unwrap();

    let file = CopcFile::open(&path).unwrap();
    assert_eq!(75.0, file.copc_info().gpstime_min);
    assert_eq!(120.0, file.copc_info().gpstime_max);

    let return_counts = read_extended_return_counts(&path);
    assert_eq!(1, return_counts[0]);
    assert_eq!(2, return_counts[1]);
    assert_eq!(1, return_counts[14]);
    assert_eq!(4, return_counts.iter().sum::<u64>());
}

#[test]
fn writer_rejects_non_finite_bounds() {
    let source = VecSource {
        points: vec![point_fields(0.0, 0.0, 0.0)],
    };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("non-finite-bounds.copc.laz");

    let err = write_source(
        &path,
        &source,
        false,
        Bounds::new((0.0, 0.0, 0.0), (f64::INFINITY, 0.0, 0.0)),
        &CopcWriterParams::default(),
    )
    .unwrap_err();

    assert!(err.to_string().contains("bounds max x must be finite"));
    assert!(!path.exists());
}

#[test]
fn writer_rejects_non_finite_source_coordinate() {
    let source = VecSource {
        points: vec![point_fields(f64::NAN, 0.0, 0.0)],
    };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("non-finite-point.copc.laz");

    let err = write_source(
        &path,
        &source,
        false,
        Bounds::point(0.0, 0.0, 0.0),
        &CopcWriterParams::default(),
    )
    .unwrap_err();

    assert!(err
        .to_string()
        .contains("point 0 x coordinate must be finite"));
    assert!(!path.exists());
}

#[test]
fn writer_rejects_non_finite_gps_time() {
    let mut point = point_fields(0.0, 0.0, 0.0);
    point.gps_time = f64::NAN;
    let source = VecSource {
        points: vec![point],
    };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("non-finite-gps-time.copc.laz");

    let err = write_source(
        &path,
        &source,
        false,
        Bounds::point(0.0, 0.0, 0.0),
        &CopcWriterParams::default(),
    )
    .unwrap_err();

    assert!(err.to_string().contains("point 0 GPS time must be finite"));
    assert!(!path.exists());
}

#[test]
fn writer_rejects_coordinate_outside_las_i32_range() {
    let source = VecSource {
        points: vec![
            point_fields(0.0, 0.0, 0.0),
            point_fields(5_000_000.0, 0.0, 0.0),
        ],
    };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("out-of-range-point.copc.laz");

    let err = write_source(
        &path,
        &source,
        false,
        Bounds::new((0.0, 0.0, 0.0), (5_000_000.0, 0.0, 0.0)),
        &CopcWriterParams::default(),
    )
    .unwrap_err();

    assert!(err.to_string().contains("outside LAS i32 range"));
    assert!(!path.exists());
}

#[test]
fn streaming_conversion_preserves_scan_angle_degrees() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("scan-angle.las");
    let copc_path = dir.path().join("scan-angle.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(6).unwrap();
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            scan_angle: 30.0,
            gps_time: Some(1.0),
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams {
            max_points_per_node: 128,
            max_depth: 4,
        },
        &spill_dir,
        &NeverCancel,
    )
    .unwrap();

    let mut reader = CopcReader::open(std::fs::File::open(&copc_path).unwrap()).unwrap();
    let points = reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .collect::<copc_core::Result<Vec<_>>>()
        .unwrap();

    assert_eq!(1, points.len());
    assert_eq!(30.0, points[0].scan_angle);
}

#[test]
fn streaming_conversion_preserves_supported_header_metadata() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("metadata.las");
    let copc_path = dir.path().join("metadata.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut builder = las::Builder::from((1, 4));
    builder.file_source_id = 42;
    builder.gps_time_type = las::GpsTimeType::Standard;
    builder.has_synthetic_return_numbers = true;
    builder.system_identifier = "source-system".to_string();
    builder.generating_software = "source-software".to_string();
    builder.point_format = las::point::Format::new(6).unwrap();
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            gps_time: Some(1.0),
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap();

    let header = read_las_header_prefix(&copc_path);
    assert_eq!(42, header.file_source_id);
    assert_eq!(25, header.global_encoding);
    assert_eq!("source-system", header.system_identifier);
    assert_eq!("source-software", header.generating_software);
}

#[test]
fn streaming_conversion_rejects_unsupported_point_dimensions() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("unsupported-dimensions.las");
    let copc_path = dir.path().join("unsupported-dimensions.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut format = las::point::Format::new(8).unwrap();
    format.extra_bytes = 2;
    let mut builder = las::Builder::from((1, 4));
    builder.point_format = format;
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            gps_time: Some(1.0),
            color: Some(Color::new(1, 2, 3)),
            nir: Some(4),
            extra_bytes: vec![5, 6],
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    let err = convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap_err();
    let message = err.to_string();
    assert!(message.contains("NIR point data"));
    assert!(message.contains("extra point byte"));
}

#[test]
fn streaming_conversion_rejects_waveform_point_dimensions() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("waveform.las");
    let copc_path = dir.path().join("waveform.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(9).unwrap();
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            gps_time: Some(1.0),
            waveform: Some(las::raw::point::Waveform {
                wave_packet_descriptor_index: 1,
                byte_offset_to_waveform_data: 0,
                waveform_packet_size_in_bytes: 1,
                return_point_waveform_location: 0.0,
                x_t: 0.0,
                y_t: 0.0,
                z_t: 0.0,
            }),
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    let err = convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap_err();

    assert!(err.to_string().contains("waveform point data"));
}

#[test]
fn streaming_conversion_rejects_source_vlrs() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("vlr.las");
    let copc_path = dir.path().join("vlr.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(6).unwrap();
    builder.vlrs.push(las::Vlr {
        user_id: "LASF_Projection".to_string(),
        record_id: 2112,
        description: "CRS metadata".to_string(),
        data: vec![1, 2, 3],
    });
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            gps_time: Some(1.0),
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    let err = convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap_err();
    assert!(err.to_string().contains("VLR"));
}

#[test]
fn streaming_conversion_allows_source_laszip_vlr() {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("compressed-source.laz");
    let copc_path = dir.path().join("compressed-source.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(6).unwrap();
    let mut writer = las::Writer::from_path(&las_path, builder.into_header().unwrap()).unwrap();
    writer
        .write(las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            return_number: 1,
            number_of_returns: 1,
            gps_time: Some(1.0),
            ..Default::default()
        })
        .unwrap();
    writer.close().unwrap();

    convert_las_to_copc_streaming(
        &las_path,
        &copc_path,
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap();
}

#[test]
fn streaming_writer_rejects_unsupported_layout_dimensions() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("streaming.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();
    let layout = StreamingLayout {
        point_format: 10,
        has_gps: true,
        has_color: true,
        has_nir: true,
        has_waveform: true,
    };

    let err = write_streaming_with_cancel(
        &path,
        layout,
        std::iter::empty::<copc_core::Result<copc_core::LasPointRecord>>(),
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap_err();

    let message = err.to_string();
    assert!(message.contains("NIR point data"));
    assert!(message.contains("waveform point data"));
}

#[test]
fn streaming_writer_rejects_non_finite_record_coordinate() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("streaming-non-finite.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();
    let layout = StreamingLayout {
        point_format: 6,
        has_gps: true,
        has_color: false,
        has_nir: false,
        has_waveform: false,
    };

    let err = write_streaming_with_cancel(
        &path,
        layout,
        vec![Ok(las_record(f64::INFINITY, 0.0, 0.0))],
        &CopcWriterParams::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap_err();

    assert!(err
        .to_string()
        .contains("point 0 x coordinate must be finite"));
    assert!(!path.exists());
}

struct LasHeaderPrefix {
    file_source_id: u16,
    global_encoding: u16,
    system_identifier: String,
    generating_software: String,
}

fn read_las_header_prefix(path: &std::path::Path) -> LasHeaderPrefix {
    let mut file = std::fs::File::open(path).unwrap();
    let mut signature = [0u8; 4];
    file.read_exact(&mut signature).unwrap();
    assert_eq!(b"LASF", &signature);
    let file_source_id = file.read_u16::<LittleEndian>().unwrap();
    let global_encoding = file.read_u16::<LittleEndian>().unwrap();
    let mut guid = [0u8; 16];
    file.read_exact(&mut guid).unwrap();
    let mut version = [0u8; 2];
    file.read_exact(&mut version).unwrap();
    let mut system_identifier = [0u8; 32];
    file.read_exact(&mut system_identifier).unwrap();
    let mut generating_software = [0u8; 32];
    file.read_exact(&mut generating_software).unwrap();

    LasHeaderPrefix {
        file_source_id,
        global_encoding,
        system_identifier: trim_nuls(&system_identifier),
        generating_software: trim_nuls(&generating_software),
    }
}

fn read_extended_return_counts(path: &std::path::Path) -> [u64; 15] {
    let mut file = std::fs::File::open(path).unwrap();
    file.seek(SeekFrom::Start(255)).unwrap();
    let mut counts = [0; 15];
    for count in &mut counts {
        *count = file.read_u64::<LittleEndian>().unwrap();
    }
    counts
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

fn assert_las_points_match_fields(expected: &[CopcPointFields], actual: &[Point]) {
    assert_eq!(expected.len(), actual.len());
    for (expected, actual) in expected.iter().zip(actual) {
        assert_eq!(expected.x, actual.x);
        assert_eq!(expected.y, actual.y);
        assert_eq!(expected.z, actual.z);
        assert_eq!(expected.intensity, actual.intensity);
        assert_eq!(expected.return_number, actual.return_number);
        assert_eq!(expected.number_of_returns, actual.number_of_returns);
        assert_eq!(expected.synthetic != 0, actual.is_synthetic);
        assert_eq!(expected.key_point != 0, actual.is_key_point);
        assert_eq!(expected.withheld != 0, actual.is_withheld);
        assert_eq!(expected.overlap != 0, actual.is_overlap);
        assert_eq!(expected.scan_channel, actual.scanner_channel);
        assert_eq!(
            expected.scan_direction_flag != 0,
            actual.scan_direction == ScanDirection::LeftToRight
        );
        assert_eq!(
            expected.edge_of_flight_line != 0,
            actual.is_edge_of_flight_line
        );
        assert_eq!(expected.classification, u8::from(actual.classification));
        assert_eq!(expected.user_data, actual.user_data);
        assert_eq!(expected.scan_angle_rank as f32, actual.scan_angle);
        assert_eq!(expected.point_source_id, actual.point_source_id);
        assert_eq!(Some(expected.gps_time), actual.gps_time);
        assert_eq!(
            Some(Color::new(expected.red, expected.green, expected.blue)),
            actual.color
        );
        assert!(actual.extra_bytes.is_empty());
        assert_eq!(None, actual.nir);
        assert_eq!(None, actual.waveform);
    }
}

fn trim_nuls(bytes: &[u8]) -> String {
    let end = bytes
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

fn point_fields(x: f64, y: f64, z: f64) -> CopcPointFields {
    CopcPointFields {
        x,
        y,
        z,
        intensity: 0,
        return_number: 1,
        number_of_returns: 1,
        synthetic: 0,
        key_point: 0,
        withheld: 0,
        overlap: 0,
        scan_channel: 0,
        scan_direction_flag: 0,
        edge_of_flight_line: 0,
        classification: 0,
        user_data: 0,
        scan_angle_rank: 0,
        point_source_id: 0,
        gps_time: 0.0,
        red: 0,
        green: 0,
        blue: 0,
    }
}

fn las_record(x: f64, y: f64, z: f64) -> copc_core::LasPointRecord {
    copc_core::LasPointRecord {
        x,
        y,
        z,
        intensity: 0,
        return_number: 1,
        number_of_returns: 1,
        classification: 0,
        scan_direction_flag: false,
        edge_of_flight_line: false,
        scan_angle: 0,
        user_data: 0,
        point_source_id: 0,
        synthetic: false,
        key_point: false,
        withheld: false,
        overlap: false,
        scan_channel: 0,
        gps_time: 0.0,
        red: 0,
        green: 0,
        blue: 0,
        nir: 0,
        wave_packet_descriptor_index: 0,
        byte_offset_to_waveform_data: 0,
        waveform_packet_size: 0,
        return_point_waveform_location: 0.0,
    }
}
