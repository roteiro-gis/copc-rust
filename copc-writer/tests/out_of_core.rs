use std::path::Path;

use copc_core::{LasPointRecord, NeverCancel, StreamingLayout};
use copc_writer::{write_streaming_with_cancel, CopcWriteMetadata, CopcWriterParams, SpillWriter};

#[test]
fn lod_index_and_spill_remain_disk_backed() {
    let writer_src = production_section(include_str!("../src/writer.rs"));
    let lod_src = production_section(include_str!("../src/lod.rs"));
    let spill_src = production_section(include_str!("../src/spill.rs"));

    assert_contains(lod_src, "struct LodIndex");
    assert_contains(lod_src, "order_path: TempPath");
    assert_contains(lod_src, "struct IndexRun");
    assert_contains(lod_src, "path: TempPath");
    assert_contains(lod_src, "fn build_lod_index");
    assert_contains(lod_src, "write_root_index_run(total_points, cancel)?");
    assert_contains(lod_src, "fn partition_index_run");
    assert_contains(lod_src, "new_index_tempfile(\"root\")");
    assert_contains(lod_src, "new_index_tempfile(\"partition\")");
    assert_contains(lod_src, "open_index_run(run)?");
    assert_contains(lod_src, "append_index_to_order(");
    assert_not_contains(lod_src, "Vec<LasPointRecord>");
    assert_not_contains(lod_src, "Vec<CopcPointFields>");
    assert_not_contains(lod_src, "Vec<u32>");
    assert_not_contains(writer_src, "Vec<LasPointRecord>");
    assert_not_contains(writer_src, "Vec<CopcPointFields>");
    assert_not_contains(writer_src, "Vec<u32>");

    assert_contains(spill_src, "scratch: Vec<u8>");
    assert_contains(spill_src, "mmap: Option<Mmap>");
    assert_contains(spill_src, "Mmap::map(&file)");
    assert_not_contains(spill_src, "records: Vec<LasPointRecord>");
    assert_not_contains(spill_src, "Vec<LasPointRecord>");

    let dir = tempfile::tempdir().unwrap();
    let layout = streaming_layout(3);
    let mut writer = SpillWriter::create(dir.path(), layout).unwrap();
    assert_eq!(1, entry_count(dir.path()));
    for index in 0..8 {
        writer.push(&record(index, 3)).unwrap();
    }

    let reader = writer.finalize().unwrap();
    assert_eq!(8, reader.len());
    assert_eq!(1, entry_count(dir.path()));
    for index in 0..8 {
        let actual = reader.record_at(index).unwrap();
        assert_eq!(record(index, 3), actual);
    }

    drop(reader);
    assert_eq!(0, entry_count(dir.path()));
}

#[cfg(any(target_os = "linux", target_os = "macos"))]
#[test]
#[ignore = "large opt-in bounded-memory conversion guard"]
fn large_streaming_conversion_peak_rss_stays_bounded() {
    let point_count = env_usize("COPC_WRITER_MEMORY_GUARD_POINTS", 4_000_000);
    let extra_bytes = env_usize("COPC_WRITER_MEMORY_GUARD_EXTRA_BYTES", 32);
    let extra_byte_width =
        u16::try_from(extra_bytes).expect("COPC_WRITER_MEMORY_GUARD_EXTRA_BYTES exceeds u16");
    let max_peak_rss_growth =
        env_usize("COPC_WRITER_MEMORY_GUARD_MAX_RSS_BYTES", 512 * 1024 * 1024) as u64;

    let dir = tempfile::tempdir().unwrap();
    let out_path = dir.path().join("large-bounded-memory.copc.laz");
    let spill_dir = dir.path().join("spill");
    std::fs::create_dir(&spill_dir).unwrap();

    let before = peak_rss_bytes();
    write_streaming_with_cancel(
        &out_path,
        streaming_layout(extra_byte_width),
        GeneratedRecords {
            next: 0,
            len: point_count,
            extra_bytes,
        },
        &CopcWriterParams::new(20_000),
        &CopcWriteMetadata::default(),
        &spill_dir,
        &NeverCancel,
    )
    .unwrap();
    let after = peak_rss_bytes();
    let peak_growth = after.saturating_sub(before);

    assert!(
        peak_growth <= max_peak_rss_growth,
        "peak RSS grew by {peak_growth} bytes for {point_count} streamed points with \
         {extra_bytes} extra bytes each; limit is {max_peak_rss_growth} bytes"
    );

    let reader = las::Reader::from_path(&out_path).unwrap();
    assert_eq!(point_count as u64, reader.header().number_of_points());
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
#[test]
#[ignore = "large opt-in bounded-memory conversion guard"]
fn large_streaming_conversion_peak_rss_stays_bounded() {
    eprintln!("getrusage peak RSS accounting is only wired for Linux and macOS");
}

struct GeneratedRecords {
    next: usize,
    len: usize,
    extra_bytes: usize,
}

impl Iterator for GeneratedRecords {
    type Item = copc_core::Result<LasPointRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next == self.len {
            return None;
        }
        let record = record(self.next, self.extra_bytes);
        self.next += 1;
        Some(Ok(record))
    }
}

fn streaming_layout(extra_bytes: u16) -> StreamingLayout {
    StreamingLayout {
        point_format: 6,
        has_gps: true,
        has_color: false,
        has_nir: false,
        has_waveform: false,
        extra_bytes,
        extra_bytes_descriptors: Vec::new(),
    }
}

fn record(index: usize, extra_byte_count: usize) -> LasPointRecord {
    let x = (index % 2048) as f64;
    let y = ((index / 2048) % 2048) as f64;
    let z = (index / (2048 * 2048)) as f64;
    let extra_bytes = (0..extra_byte_count)
        .map(|byte| ((index.wrapping_mul(31) + byte) & 0xff) as u8)
        .collect();

    LasPointRecord {
        x,
        y,
        z,
        intensity: index as u16,
        return_number: 1,
        number_of_returns: 1,
        classification: 1,
        scan_direction_flag: false,
        edge_of_flight_line: false,
        scan_angle: 0.0,
        user_data: 0,
        point_source_id: 0,
        synthetic: false,
        key_point: false,
        withheld: false,
        overlap: false,
        scan_channel: 0,
        gps_time: index as f64 * 0.001,
        red: 0,
        green: 0,
        blue: 0,
        nir: 0,
        wave_packet_descriptor_index: 0,
        byte_offset_to_waveform_data: 0,
        waveform_packet_size: 0,
        return_point_waveform_location: 0.0,
        extra_bytes,
    }
}

fn production_section(src: &'static str) -> &'static str {
    src.split("\n#[cfg(test)]\nmod tests").next().unwrap_or(src)
}

fn assert_contains(haystack: &str, needle: &str) {
    assert!(
        haystack.contains(needle),
        "expected production source to contain `{needle}`"
    );
}

fn assert_not_contains(haystack: &str, needle: &str) {
    assert!(
        !haystack.contains(needle),
        "expected production source not to contain `{needle}`"
    );
}

fn entry_count(path: &Path) -> usize {
    std::fs::read_dir(path).unwrap().count()
}

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

#[cfg(any(target_os = "linux", target_os = "macos"))]
fn peak_rss_bytes() -> u64 {
    use std::mem::MaybeUninit;
    use std::os::raw::{c_int, c_long};

    #[repr(C)]
    struct TimeVal {
        tv_sec: c_long,
        tv_usec: c_long,
    }

    #[repr(C)]
    struct RUsage {
        ru_utime: TimeVal,
        ru_stime: TimeVal,
        ru_maxrss: c_long,
        ru_ixrss: c_long,
        ru_idrss: c_long,
        ru_isrss: c_long,
        ru_minflt: c_long,
        ru_majflt: c_long,
        ru_nswap: c_long,
        ru_inblock: c_long,
        ru_oublock: c_long,
        ru_msgsnd: c_long,
        ru_msgrcv: c_long,
        ru_nsignals: c_long,
        ru_nvcsw: c_long,
        ru_nivcsw: c_long,
    }

    extern "C" {
        fn getrusage(who: c_int, usage: *mut RUsage) -> c_int;
    }

    const RUSAGE_SELF: c_int = 0;

    let mut usage = MaybeUninit::<RUsage>::zeroed();
    let rc = unsafe { getrusage(RUSAGE_SELF, usage.as_mut_ptr()) };
    assert_eq!(0, rc, "getrusage failed");
    let usage = unsafe { usage.assume_init() };
    let maxrss = usage.ru_maxrss as u64;
    if cfg!(target_os = "linux") {
        maxrss * 1024
    } else {
        maxrss
    }
}
