//! Fuzz point decoding: open, iterate a bounded number of rows, and run one
//! column read over arbitrary bytes.

#![no_main]

use copc_core::ColumnSelection;
use copc_reader::{BoundsSelection, CopcReader, LodSelection, PointQuery};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Ok(mut reader) = CopcReader::open(std::io::Cursor::new(data)) else {
        return;
    };
    if let Ok(points) = reader.points(LodSelection::All, BoundsSelection::All) {
        for point in points.take(10_000) {
            if point.is_err() {
                break;
            }
        }
    }
    let _ = reader.read_columns(PointQuery::all(), ColumnSelection::xyz());
});
