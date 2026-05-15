# External COPC Fixtures

Place third-party COPC files here to run reader conformance against files not
written by `copc-writer`.

Expected layout:

```text
copc-reader/tests/fixtures/external/
  pdal/*.copc.laz
  qgis/*.copc.laz
```

Run with:

```sh
cargo test -p copc-reader --test external_fixtures -- --ignored
```

Keep fixtures small enough for source control or stage them through CI artifact
download before running the ignored test. The test opens each file, validates
header/hierarchy point counts, iterates all points, checks full-file bounds
selection, and verifies root LOD access.
