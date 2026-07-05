# copc-rust

[![copc-core crates.io](https://img.shields.io/crates/v/copc-core.svg)](https://crates.io/crates/copc-core)
[![copc-core docs.rs](https://docs.rs/copc-core/badge.svg)](https://docs.rs/copc-core)
[![copc-reader crates.io](https://img.shields.io/crates/v/copc-reader.svg)](https://crates.io/crates/copc-reader)
[![copc-reader docs.rs](https://docs.rs/copc-reader/badge.svg)](https://docs.rs/copc-reader)
[![copc-writer crates.io](https://img.shields.io/crates/v/copc-writer.svg)](https://crates.io/crates/copc-writer)
[![copc-writer docs.rs](https://docs.rs/copc-writer/badge.svg)](https://docs.rs/copc-writer)

Pure-Rust COPC reader, writer, and shared core primitives for cloud-optimized
point clouds. No C libraries, no build scripts; internal unsafe is limited to
read-only memory mapping of writer spill files.

## Crates

| Crate | Description |
|---|---|
| `copc-core` | Shared COPC metadata, hierarchy entries, voxel keys, bounds, LAS-native column batches, streaming LAS records, and errors |
| `copc-reader` | COPC header/info parsing, recursive hierarchy access, chunked-LAZ row iteration, and materialized column reads |
| `copc-writer` | COPC writer with source-trait point access, column-batch source support, native LOD distribution, mmap spill support, and streaming LAS/LAZ intake |

## Usage

```rust
use copc_reader::CopcFile;

let file = CopcFile::open("cloud.copc.laz")?;
for entry in file.hierarchy_walk() {
    println!("{:?} points={}", entry.key, entry.point_count);
}
```

```rust
use copc_reader::{BoundsSelection, CopcReader, LodSelection};

let mut reader = CopcReader::from_path("cloud.copc.laz")?;
for point in reader.points(LodSelection::All, BoundsSelection::All)? {
    let point = point?;
    println!("{},{},{}", point.x, point.y, point.z);
}
```

```rust
use copc_core::{ColumnData, LasDimension};
use copc_reader::{ColumnSelection, CopcReader, PointQuery};

let mut reader = CopcReader::from_path("cloud.copc.laz")?;
let batch = reader.read_columns(
    PointQuery::all(),
    ColumnSelection::from_dimensions([
        LasDimension::X,
        LasDimension::Y,
        LasDimension::Z,
        LasDimension::Classification,
    ]),
)?;

if let Some(ColumnData::F64(xs)) = batch.column(LasDimension::X) {
    println!("decoded {} x coordinates", xs.len());
}
```

```rust
use copc_writer::{convert_las_to_copc_streaming, CopcWriterParams};

convert_las_to_copc_streaming(
    "input.laz".as_ref(),
    "output.copc.laz".as_ref(),
    &CopcWriterParams::default(),
    std::env::temp_dir().as_ref(),
    &copc_core::NeverCancel,
)?;
```

For GeoTIFF-only CRS inputs, resolve CRS externally and call
`convert_las_to_copc_streaming_with_crs_wkt_override` with `Some(wkt)`; the
writer emits the supplied WKT CRS VLR without depending on a geodesy library.

## Column Ownership Model

`copc-core` owns the LAS/COPC-native column model: `LasDimension`,
`ColumnSpec`, `ColumnData`, `ColumnView`, `ColumnSelection`, and
`LasColumnBatch`. These types are dependency-light and do not depend on Arrow,
DataFusion, or engine-specific point-cloud crates.

`copc-reader` exposes materialized column batches with
`CopcReader::read_columns` and `CopcReader::read_columns_with_cancel`.
Existing row iteration with `points`, `points_for_query`, and
`points_with_cancel` remains supported.

The column API is materialized. COPC point data is still read from compressed
LAZ chunks, decoded, filtered, transformed, and appended into owned column
buffers. It is not a zero-copy view into compressed COPC files.

Downstream engines should adapt `LasColumnBatch` into their own canonical
memory model. For example, `roteiro-engine` maps these native batches into its
`PointCloud` struct-of-arrays representation. Arrow conversion is intentionally
out of scope for `copc-rust` today; it belongs in downstream engine code or
behind a future optional feature.

## Supported Now

- Public COPC hierarchy types for availability, indexing, and tile serving
- COPC info VLR and recursive hierarchy page parsing
- Chunked-LAZ point iteration in `copc-reader`
- All-points, LOD-selected, and bounds-selected reader point iteration
- Materialized LAS/COPC-native column batches in `copc-reader`
- Source-trait writer API for caller-owned point storage
- COPC writing from neutral `LasColumnBatch` values via `ColumnBatchSource`
- Streaming LAS/LAZ-to-COPC conversion through a disk-backed mmap spill
- Streaming conversion preserves WKT CRS records, LAS Extra Bytes payloads and
  descriptors, and source non-CRS VLRs/EVLRs
- Streaming conversion can accept caller-resolved WKT for GeoTIFF-only CRS
  inputs without adding a geodesy dependency to `copc-writer`
- LAS 1.4 point formats 6 and 7 with LAZ variable-size chunks
- Interior-node representative points for native LOD reads

## Not Yet Supported

- Zero-copy column views directly over compressed COPC/LAZ point data
- Built-in Arrow or DataFusion conversion
- Built-in GeoTIFF-only CRS conversion to WKT during LAS/LAZ-to-COPC conversion

## Testing

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

Checked-in external COPC fixtures from PDAL and QGIS are exercised by:

```sh
cargo test -p copc-reader --test external_fixtures
```

## License

MIT OR Apache-2.0
