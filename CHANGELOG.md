# Changelog

## Unreleased

## 0.4.2 - 2026-07-05

- add a proj-free CRS WKT override hook for LAS/LAZ-to-COPC streaming
  conversion, allowing downstream engines to resolve GeoTIFF-only CRS metadata
  externally and have `copc-writer` emit the resulting WKT CRS VLR
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
