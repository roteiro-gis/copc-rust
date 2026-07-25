//! Read-path benchmarks over a synthetic writer-generated COPC file.

use std::path::Path;

use copc_core::{Bounds, ColumnSelection, LasDimension, Result};
use copc_reader::{BoundsSelection, CopcReader, LodSelection, PointQuery};
use copc_writer::{
    write_source, CopcPointFields, CopcPointSource, CopcWriteMetadata, CopcWriterParams,
};
use criterion::{criterion_group, criterion_main, Criterion, Throughput};

const POINT_COUNT: usize = 1_000_000;

struct SyntheticSource {
    count: usize,
}

impl SyntheticSource {
    fn point(&self, index: usize) -> CopcPointFields {
        let i = index as f64;
        let mut fields = CopcPointFields {
            x: (index % 1_000) as f64 * 0.75,
            y: ((index / 1_000) % 1_000) as f64 * 0.75,
            z: (i * 0.001).sin() * 50.0 + 100.0,
            intensity: (index % 65_536) as u16,
            return_number: (index % 4 + 1) as u8,
            number_of_returns: 4,
            classification: (index % 18) as u8,
            gps_time: 1.0e9 + i * 1e-4,
            point_source_id: (index % 512) as u16,
            user_data: (index % 251) as u8,
            scan_angle: ((index % 61) as f32 - 30.0) * 0.006,
            ..CopcPointFields::default()
        };
        fields.scan_channel = (index % 4) as u8;
        fields
    }
}

impl CopcPointSource for SyntheticSource {
    fn len(&self) -> usize {
        self.count
    }

    fn xyz(&self, index: usize) -> Result<(f64, f64, f64)> {
        let p = self.point(index);
        Ok((p.x, p.y, p.z))
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> Result<()> {
        *out = self.point(index);
        Ok(())
    }
}

fn write_fixture(path: &Path, count: usize) {
    let source = SyntheticSource { count };
    let mut bounds = Bounds::point(0.0, 0.0, 0.0);
    for index in 0..count {
        let (x, y, z) = source.xyz(index).unwrap();
        bounds.extend(x, y, z);
    }
    write_source(
        path,
        &source,
        false,
        bounds,
        &CopcWriterParams::default(),
        &CopcWriteMetadata::default(),
    )
    .unwrap();
}

fn bench_read(c: &mut Criterion) {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bench.copc.laz");
    write_fixture(&path, POINT_COUNT);

    let mut group = c.benchmark_group("read");
    group.throughput(Throughput::Elements(POINT_COUNT as u64));
    group.sample_size(10);

    group.bench_function("points_all", |b| {
        b.iter(|| {
            let mut reader = CopcReader::from_path(&path).unwrap();
            let count = reader
                .points(LodSelection::All, BoundsSelection::All)
                .unwrap()
                .fold(0usize, |count, point| {
                    point.unwrap();
                    count + 1
                });
            assert_eq!(POINT_COUNT, count);
        })
    });

    group.bench_function("read_columns_all", |b| {
        b.iter(|| {
            let mut reader = CopcReader::from_path(&path).unwrap();
            let batch = reader
                .read_columns(PointQuery::all(), ColumnSelection::all())
                .unwrap();
            assert_eq!(POINT_COUNT, batch.len());
        })
    });

    group.bench_function("read_columns_xyz", |b| {
        b.iter(|| {
            let mut reader = CopcReader::from_path(&path).unwrap();
            let batch = reader
                .read_columns(PointQuery::all(), ColumnSelection::xyz())
                .unwrap();
            assert_eq!(POINT_COUNT, batch.len());
            assert!(batch.column(LasDimension::X).is_some());
        })
    });

    group.finish();
}

criterion_group!(benches, bench_read);
criterion_main!(benches);
