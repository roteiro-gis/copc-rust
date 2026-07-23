//! Fuzz COPC metadata parsing: header, VLRs, EVLRs, and recursive hierarchy.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = copc_reader::CopcFile::from_reader(&mut std::io::Cursor::new(data));
});
