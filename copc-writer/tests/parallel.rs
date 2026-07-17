//! Feature-gated guard: parallel chunk compression must produce output that
//! is reader-visible-identical to the source (same points, per-node chunks,
//! and a chunk table that `las::Reader` accepts).

#![cfg(feature = "parallel")]

use copc_core::Bounds;
use copc_reader::{BoundsSelection, CopcFile, CopcReader, LodSelection};
use copc_writer::{
    write_source, CopcPointFields, CopcPointSource, CopcWriteMetadata, CopcWriterParams,
};
use las::Read as _;

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

#[test]
fn parallel_write_round_trips_multi_chunk_cloud() {
    let mut points = Vec::new();
    for i in 0..20_000u32 {
        let f = f64::from(i);
        points.push(CopcPointFields {
            x: (f * 37.0) % 509.0,
            y: (f * 53.0) % 521.0,
            z: (f * 71.0) % 127.0,
            intensity: (i % 65_536) as u16,
            return_number: (i % 4 + 1) as u8,
            number_of_returns: 4,
            classification: (i % 18) as u8,
            gps_time: 1.0e9 + f * 1e-3,
            scan_angle: ((i % 61) as f32 - 30.0) * 0.006,
            point_source_id: (i % 512) as u16,
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
    let expected_gps: Vec<f64> = points.iter().map(|point| point.gps_time).collect();
    let source = VecSource { points };
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("parallel.copc.laz");

    write_source(
        &path,
        &source,
        false,
        bounds,
        &CopcWriterParams::new(1_000),
        &CopcWriteMetadata::default(),
    )
    .unwrap();

    // Multiple chunks were emitted, one per octree node.
    let file = CopcFile::open(&path).unwrap();
    let entries = file.hierarchy_walk();
    assert!(entries.len() > 4, "expected multiple chunks");
    assert_eq!(
        source.len(),
        entries
            .iter()
            .map(|entry| entry.point_count as usize)
            .sum::<usize>()
    );

    // COPC reader sees every point.
    let mut reader = CopcReader::from_path(&path).unwrap();
    let mut gps: Vec<f64> = reader
        .points(LodSelection::All, BoundsSelection::All)
        .unwrap()
        .map(|point| point.unwrap().gps_time.unwrap())
        .collect();
    gps.sort_by(f64::total_cmp);
    let mut expected = expected_gps.clone();
    expected.sort_by(f64::total_cmp);
    assert_eq!(expected, gps);

    // The chunk table written by the parallel path must satisfy a plain
    // LAZ reader doing sequential decompression.
    let mut las_reader = las::Reader::from_path(&path).unwrap();
    assert_eq!(source.len() as u64, las_reader.header().number_of_points());
    let mut las_gps: Vec<f64> = las_reader
        .points()
        .map(|point| point.unwrap().gps_time.unwrap())
        .collect();
    las_gps.sort_by(f64::total_cmp);
    assert_eq!(expected, las_gps);
}
