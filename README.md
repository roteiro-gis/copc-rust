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
| `copc-core` | Shared COPC metadata, hierarchy entries, voxel keys, bounds, streaming LAS records, and errors |
| `copc-reader` | COPC header, info VLR, and hierarchy parsing with public hierarchy access |
| `copc-writer` | COPC writer with source-trait point access, native LOD distribution, mmap spill support, and streaming LAS/LAZ intake |

## Usage

```rust
use copc_reader::CopcFile;

let file = CopcFile::open("cloud.copc.laz")?;
for entry in file.hierarchy_walk() {
    println!("{:?} points={}", entry.key, entry.point_count);
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

## Supported Now

- Public COPC hierarchy types for availability, indexing, and tile serving
- COPC info VLR and root hierarchy EVLR parsing
- Source-trait writer API for caller-owned point storage
- Streaming LAS/LAZ-to-COPC conversion through a disk-backed mmap spill
- LAS 1.4 point formats 6 and 7 with LAZ variable-size chunks
- Interior-node representative points for native LOD reads

## Not Yet Supported

- Chunked-LAZ point iteration in `copc-reader`
- Recursive child hierarchy page loading beyond the root page
- Bounds/LOD-selected reader point iteration
- Materialized point-column convenience APIs

## Testing

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## License

MIT OR Apache-2.0
