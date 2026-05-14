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
| `copc-writer` | COPC writer with LOD node distribution, source-trait point access, mmap spill support, and streaming LAS/LAZ intake |

## Reading

```rust
use copc_reader::CopcFile;

let file = CopcFile::open("cloud.copc.laz")?;
println!("points: {}", file.header().number_of_points);
println!("root hierarchy bytes: {}", file.copc_info().root_hier_size);

for entry in file.hierarchy_walk() {
    println!("{:?} points={}", entry.key, entry.point_count);
}
```

## Writing

```rust
use copc_core::Bounds;
use copc_writer::{write_source, CopcPointFields, CopcPointSource, CopcWriterParams};

struct Source {
    points: Vec<CopcPointFields>,
}

impl CopcPointSource for Source {
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

let source = Source { points };
write_source(
    "out.copc.laz".as_ref(),
    &source,
    false,
    Bounds::new((0.0, 0.0, 0.0), (100.0, 100.0, 30.0)),
    &CopcWriterParams::default(),
)?;
```

For out-of-core LAS/LAZ conversion, use `convert_las_to_copc_streaming()`.
The converter spills full-fidelity LAS records to an explicit little-endian
temporary file and memory maps that file while building the COPC hierarchy and
LAZ chunks.

## Supported Now

- Public COPC core types: `CopcInfo`, `HierarchyPage`, `Entry`, and `VoxelKey`
- COPC info VLR and root hierarchy EVLR parse/serialize helpers
- LAS point-format-aware streaming record layout and explicit little-endian
  record serialization
- Source-trait COPC writer for caller-owned point storage
- Streaming LAS/LAZ-to-COPC conversion through a disk-backed mmap spill
- COPC writer output with LAS 1.4 point formats 6 and 7, LAZ variable-size
  chunks, COPC info VLR, and hierarchy EVLR

## Not Yet Supported

- Chunked-LAZ point iteration in `copc-reader`
- Recursive child hierarchy page loading beyond the root page
- Bounds/LOD-selected reader point iteration
- Materialized point-column convenience APIs

COPC is sparse point data rather than a dense grid. The primary API is
streaming/chunk/hierarchy-first; any future `ndarray` convenience should be an
optional materialization layer, not a core dependency.

## Testing

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## License

MIT OR Apache-2.0
