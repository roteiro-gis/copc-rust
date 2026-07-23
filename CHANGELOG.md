# Changelog

## Unreleased

## 0.5.0 - 2026-07-10

Security hardening, breaking API corrections, hot-path performance work,
optional parallelism, and a lazy remote range reader.

### Security

- load hierarchy pages with an explicit worklist instead of recursion, so a
  crafted file with a deep chain of child pages can no longer overflow the
  stack
- reject files whose hierarchy point totals exceed the LAS header point count,
  and cap the column capacity reserved up front from untrusted chunk counts
- cap source EVLR counts read during LAS/LAZ-to-COPC conversion at 4,096,
  matching the reader's limit (shared via `copc_core::limits`)
- reject `write_source` points that lie outside the declared bounds; they
  would otherwise land in octree voxels whose bounds don't contain them and
  silently vanish from readers' spatial queries
- add cargo-fuzz targets for COPC open and point reads with a CI smoke run,
  plus a cargo-audit CI job

### Breaking

- `LasDimension::ScanAngleRank` (i16, degrees×2) is now
  `LasDimension::ScanAngle` (F32, degrees). Column reads and
  `ColumnBatchSource` writes carry scan angles losslessly to LAS 1.4's 0.006°
  resolution; `copc_core::scan_angle_rank_from_degrees` is removed. Migrate
  columns by multiplying old rank values by 0.5 to get degrees.
- `CopcWriterParams` loses `max_depth` (it was silently clamped to an internal
  21-30 range and never behaved as documented) and is `#[non_exhaustive]`;
  construct it with `CopcWriterParams::new(max_points_per_node)` or
  `::default()`.
- `write_source`, `write_source_with_cancel`, and
  `write_streaming_with_cancel` take a new `&CopcWriteMetadata` parameter for
  WKT CRS, GUID, identifiers, creation date, GPS time type, and scale/offset
  overrides; pass `&CopcWriteMetadata::default()` to keep prior behavior.
- `CopcPointSource::fields` is now
  `fields_into(&self, index, out: &mut CopcPointFields)`, reusing the output's
  allocations across points; `CopcPointFields` gains `Default`.
- generated PDRF 6/7 files now always set the LAS WKT global-encoding bit and
  carry a WKT CRS VLR (empty when no CRS is supplied), as LAS 1.4 requires;
  header creation dates default to the current UTC date instead of a
  hardcoded year.
- `SpillWriter::push` now validates records (coordinates, scan angle, LAS
  field ranges, GPS time) at intake and accumulates output statistics, so
  spill-backed writes skip the second full validation pass.

### Added

- `CopcRangeReader` in `copc-reader`: a lazy reader over a new `RangeRead`
  byte-range trait that fetches the header and VLRs at open, loads hierarchy
  pages only when a query's LOD/bounds can reach them, coalesces adjacent
  chunk fetches (64 KiB gap threshold), and exposes `hierarchy_for`,
  `read_points`, and `read_columns`
- `HttpRangeReader` behind the `http` feature (plain HTTP; enable `http-tls`
  for HTTPS) using HTTP `Range` requests, verified against a local
  range-serving test server
- optional `parallel` feature in both `copc-writer` (rayon-parallel per-node
  LAZ chunk compression with a hand-assembled chunk table that plain LAZ
  readers accept) and `copc-reader` (parallel chunk decoding for column reads)
- criterion benchmarks for reader row/column throughput and streaming
  conversion

### Fixed

- writer scan angles are now rounded to the nearest LAS 1.4 0.006° increment
  instead of truncated (up to a full increment of error via
  `las::raw::point::ScanAngle::from(f32)`)
- failed writes no longer leave a partial output file: output is written to a
  same-directory temp file and atomically renamed on success

### Performance

- column reads decode fields directly from decompressed record bytes instead
  of materializing a `las::raw::Point` per point (~9% faster full-column
  reads; column reads now run at the raw LAZ-decode floor)
- point encoding writes the PDRF 6/7 layout directly, eliminating two
  per-point heap allocations in the compression loop (guarded by a
  byte-identity test against `las::raw::Point`)
- spill-backed writes validate once at intake instead of re-deserializing
  every record in a second pass; per-point `format!` allocations removed from
  the stats path

## 0.4.2 - 2026-07-05

- add a proj-free CRS WKT override hook for LAS/LAZ-to-COPC streaming
  conversion, allowing downstream engines to resolve GeoTIFF-only CRS metadata
  externally and have `copc-writer` emit the resulting WKT CRS VLR
- null-terminate caller-supplied WKT override CRS VLR payloads for stricter
  LAS/COPC reader interoperability
- keep GeoTIFF-only CRS inputs rejected with the specific unsupported
  conversion error when no WKT CRS record or WKT override is supplied

## 0.4.1 - 2026-07-04

- preserve source EVLRs when converting from COPC/LAZ inputs whose EVLR list
  extends beyond the first COPC hierarchy record
- add a compressed COPC/LAZ-input round-trip gate covering WKT CRS, LAS Extra
  Bytes, a non-CRS VLR, and a source EVLR
- increase bounded spill, index, source, and output I/O buffers used by
  streaming conversion

## 0.4.0 - 2026-07-04

- preserve WKT CRS records during LAS/LAZ-to-COPC conversion and set the LAS
  WKT global-encoding bit when a WKT CRS is carried through
- preserve LAS Extra Bytes point payloads and LASF_Spec Extra Bytes descriptor
  VLRs through streaming conversion, disk spill, LAZ encoding, and read-back
- pass through source non-CRS VLRs and EVLRs while regenerating COPC info,
  LASzip, CRS, hierarchy, and Extra Bytes records in the required output order
- keep streaming conversion out-of-core by carrying Extra Bytes through the
  spill file and retaining the temp-file-backed LOD index path
- add release gates for combined CRS, Extra Bytes, VLR, and EVLR round-trips,
  plus structural and opt-in large-N bounded-memory writer guards
- reject GeoTIFF-only CRS inputs with a specific unsupported-conversion error
  until GeoTIFF-to-WKT conversion is implemented
- breaking: `LasPointRecord` now carries raw `extra_bytes`, and
  `StreamingLayout` now carries `extra_bytes` plus Extra Bytes descriptor VLRs

## 0.3.0 - 2026-06-22

- add LAS/COPC-native column types in `copc-core`, materialized column reads in
  `copc-reader`, and a `copc-writer` `ColumnBatchSource` adapter for writing
  neutral `LasColumnBatch` values directly
- model LAS Extra Bytes columns as fixed-width byte data with explicit
  per-point width metadata, so `ColumnSelection::all()` and point-format
  layouts can include Extra Bytes without rejecting valid multi-byte records
- keep row iteration supported while documenting that column reads are owned,
  materialized buffers decoded from compressed LAZ chunks, not zero-copy views
  into COPC files
- keep Arrow/DataFusion conversion out of `copc-rust`; downstream engines can
  adapt `LasColumnBatch` into their own models or a future optional feature can
  add Arrow-specific conversion

## 0.2.0 - 2026-06-10

- reject COPC files whose VLR/EVLR sections, hierarchy pages, or child hierarchy pages extend past EOF; cap VLR/EVLR counts at 4,096, one hierarchy page at 64 MiB, and recursively loaded hierarchy pages at 256 MiB; add truncation tests that assert errors instead of panics
- make writer spill files private on Unix, ensure finalized and cancelled spill files are removed, and move `tempfile` into runtime dependencies
- reject unsupported LAS-to-COPC conversions before lossy output, including NIR, waveform data, Extra Bytes, source VLRs/EVLRs, and padding that cannot yet be preserved
- preserve source LAS metadata supported by the writer, including file source id, global encoding GPS time type, project GUID, generating software, creation date, scale, and offset
- preserve fractional LAS 1.4 scan angles through streaming conversion and COPC writes
- validate finite coordinates, quantized coordinate ranges, scan-angle encoding ranges, GPS time, bounds, transforms, and LAS point flag/channel fields before writing
- store writer LOD indexes in spill-backed index files and split hierarchy output into 4,096-entry pages
- emit COPC GPS time ranges and extended return histograms in output metadata
- add COPC reader/writer conformance, malformed-input, field round-trip, and streaming-conversion regression tests
- pin CI runner/toolchain versions, add an MSRV check, and verify package/publish dry-runs in CI
- clear the LAS WKT global-encoding bit for generated files until CRS VLR preservation is implemented

## 0.1.0 - 2026-05-15

- initial public release
- add `copc-core` shared COPC metadata, hierarchy entries, voxel keys, bounds, streaming LAS records, and errors
- add `copc-reader` COPC header/info parsing, recursive hierarchy access, and chunked-LAZ point iteration
- add `copc-writer` source-trait point access, native LOD distribution, mmap spill support, and streaming LAS/LAZ input
- support all-points, LOD-selected, and bounds-selected reader point iteration
- support LAS 1.4 point formats 6 and 7 with LAZ variable-size chunks
- add interior-node representative points for native LOD reads
