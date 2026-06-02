# Changelog

## Unreleased

## 0.1.0 - 2026-05-15

- initial public release
- add `copc-core` shared COPC metadata, hierarchy entries, voxel keys, bounds, streaming LAS records, and errors
- add `copc-reader` COPC header/info parsing, recursive hierarchy access, and chunked-LAZ point iteration
- add `copc-writer` source-trait point access, native LOD distribution, mmap spill support, and streaming LAS/LAZ input
- support all-points, LOD-selected, and bounds-selected reader point iteration
- support LAS 1.4 point formats 6 and 7 with LAZ variable-size chunks
- add interior-node representative points for native LOD reads
