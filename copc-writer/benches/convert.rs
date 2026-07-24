//! Streaming LAS-to-COPC conversion benchmark.

use std::path::Path;

use copc_writer::{convert_las_to_copc_streaming, CopcWriterParams};
use criterion::{criterion_group, criterion_main, Criterion, Throughput};

const POINT_COUNT: usize = 250_000;

fn write_source_las(path: &Path, count: usize) {
    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(6).unwrap();
    let mut writer = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
    for index in 0..count {
        let i = index as f64;
        writer
            .write_point(las::Point {
                x: (index % 1_000) as f64 * 0.75,
                y: ((index / 1_000) % 1_000) as f64 * 0.75,
                z: (i * 0.001).sin() * 50.0 + 100.0,
                intensity: (index % 65_536) as u16,
                return_number: (index % 4 + 1) as u8,
                number_of_returns: 4,
                gps_time: Some(1.0e9 + i * 1e-4),
                ..Default::default()
            })
            .unwrap();
    }
    writer.close().unwrap();
}

fn bench_convert(c: &mut Criterion) {
    let dir = tempfile::tempdir().unwrap();
    let las_path = dir.path().join("bench-source.las");
    write_source_las(&las_path, POINT_COUNT);

    let mut group = c.benchmark_group("convert");
    group.throughput(Throughput::Elements(POINT_COUNT as u64));
    group.sample_size(10);

    group.bench_function("las_to_copc_streaming", |b| {
        b.iter(|| {
            let out_path = dir.path().join("bench-out.copc.laz");
            convert_las_to_copc_streaming(
                &las_path,
                &out_path,
                &CopcWriterParams::default(),
                dir.path(),
                &copc_core::NeverCancel,
            )
            .unwrap();
        })
    });

    group.finish();
}

criterion_group!(benches, bench_convert);
criterion_main!(benches);
