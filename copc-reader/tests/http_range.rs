//! End-to-end HTTP range reading against a local server.

#![cfg(feature = "http")]

use std::sync::Arc;

use copc_core::Bounds;
use copc_reader::{ColumnSelection, CopcRangeReader, HttpRangeReader, LasDimension, PointQuery};
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

    fn xyz(&self, index: usize) -> copc_core::Result<(f64, f64, f64)> {
        let p = &self.points[index];
        Ok((p.x, p.y, p.z))
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> copc_core::Result<()> {
        out.clone_from(&self.points[index]);
        Ok(())
    }
}

fn fixture_bytes() -> Vec<u8> {
    let mut points = Vec::new();
    for i in 0..3_000u32 {
        let f = f64::from(i);
        points.push(CopcPointFields {
            x: (f * 37.0) % 211.0,
            y: (f * 53.0) % 223.0,
            z: (f * 71.0) % 47.0,
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
    let path = dir.path().join("http.copc.laz");
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

/// Serves `data` with HTTP Range support on a local port.
fn serve_with_ranges(data: Vec<u8>) -> String {
    let server = tiny_http::Server::http("127.0.0.1:0").unwrap();
    let port = server.server_addr().to_ip().unwrap().port();
    let data = Arc::new(data);
    std::thread::spawn(move || {
        for request in server.incoming_requests() {
            let range = request
                .headers()
                .iter()
                .find(|header| header.field.equiv("Range"))
                .map(|header| header.value.as_str().to_string());
            match range.and_then(|value| parse_range(&value, data.len() as u64)) {
                Some((start, end_inclusive)) => {
                    let body = data[start as usize..=end_inclusive as usize].to_vec();
                    let content_range =
                        format!("bytes {start}-{end_inclusive}/{total}", total = data.len());
                    let response = tiny_http::Response::from_data(body)
                        .with_status_code(206)
                        .with_header(
                            tiny_http::Header::from_bytes(
                                &b"Content-Range"[..],
                                content_range.as_bytes(),
                            )
                            .unwrap(),
                        );
                    let _ = request.respond(response);
                }
                None => {
                    let _ = request.respond(tiny_http::Response::from_data(data.to_vec()));
                }
            }
        }
    });
    format!("http://127.0.0.1:{port}/fixture.copc.laz")
}

fn parse_range(value: &str, total: u64) -> Option<(u64, u64)> {
    let spec = value.strip_prefix("bytes=")?;
    let (start, end) = spec.split_once('-')?;
    let start: u64 = start.parse().ok()?;
    let end_inclusive: u64 = end.parse().ok()?;
    (start <= end_inclusive && end_inclusive < total).then_some((start, end_inclusive))
}

#[test]
fn http_range_reader_round_trips_over_local_server() {
    let bytes = fixture_bytes();
    let expected_len = 3_000usize;
    let url = serve_with_ranges(bytes);

    let mut reader = CopcRangeReader::open(HttpRangeReader::new(url)).unwrap();

    let points = reader.read_points(PointQuery::all()).unwrap();
    assert_eq!(expected_len, points.len());

    let batch = reader
        .read_columns(PointQuery::all(), ColumnSelection::xyz())
        .unwrap();
    assert_eq!(expected_len, batch.len());
    assert!(batch.column(LasDimension::X).is_some());
}
