//! Disk spill for streaming COPC writes.

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use copc_core::{
    deserialize_le, serialize_le, Bounds, Error, LasPointRecord, Result, StreamingLayout,
};
use memmap2::Mmap;

static SPILL_SEQUENCE: AtomicU64 = AtomicU64::new(0);

/// Streams `LasPointRecord` values to a process-local temporary spill file.
pub struct SpillWriter {
    path: PathBuf,
    file: Option<BufWriter<File>>,
    layout: StreamingLayout,
    record_width: usize,
    scratch: Vec<u8>,
    count: u64,
    bounds: Option<Bounds>,
    keep_file: bool,
}

impl SpillWriter {
    pub fn create(spill_dir: &Path, layout: StreamingLayout) -> Result<Self> {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        let sequence = SPILL_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let name = format!(
            ".roteiro-copc-spill.{}.{}.{}.part",
            std::process::id(),
            nanos,
            sequence
        );
        let path = spill_dir.join(name);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| Error::io("create spill file", e))?;
        let record_width = layout.record_width();
        Ok(Self {
            path,
            file: Some(BufWriter::new(file)),
            layout,
            record_width,
            scratch: vec![0u8; record_width],
            count: 0,
            bounds: None,
            keep_file: false,
        })
    }

    pub fn push(&mut self, record: &LasPointRecord) -> Result<()> {
        serialize_le(record, &self.layout, &mut self.scratch);
        let writer = self
            .file
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("spill writer already finalized".into()))?;
        writer
            .write_all(&self.scratch)
            .map_err(|e| Error::io("write spill record", e))?;
        match self.bounds.as_mut() {
            Some(bounds) => bounds.extend(record.x, record.y, record.z),
            None => self.bounds = Some(Bounds::point(record.x, record.y, record.z)),
        }
        self.count += 1;
        Ok(())
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn finalize(mut self) -> Result<SpillReader> {
        let mut writer = self
            .file
            .take()
            .ok_or_else(|| Error::InvalidInput("spill writer already finalized".into()))?;
        writer
            .flush()
            .map_err(|e| Error::io("flush spill writer", e))?;
        let file = writer
            .into_inner()
            .map_err(|e| Error::io("unwrap spill writer", e.into_error()))?;
        file.sync_all()
            .map_err(|e| Error::io("sync spill file", e))?;
        self.keep_file = true;
        let path = self.path.clone();
        let count = usize::try_from(self.count)
            .map_err(|_| Error::InvalidInput("spill record count exceeds usize range".into()))?;
        let bounds = self.bounds.unwrap_or_else(|| Bounds::point(0.0, 0.0, 0.0));
        SpillReader::open(path, self.layout, self.record_width, count, bounds)
    }
}

impl Drop for SpillWriter {
    fn drop(&mut self) {
        if !self.keep_file {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

/// Memory-mapped random-access view over a finalized spill file.
pub struct SpillReader {
    path: PathBuf,
    _file: File,
    mmap: Mmap,
    layout: StreamingLayout,
    record_width: usize,
    count: usize,
    bounds: Bounds,
}

impl SpillReader {
    fn open(
        path: PathBuf,
        layout: StreamingLayout,
        record_width: usize,
        count: usize,
        bounds: Bounds,
    ) -> Result<Self> {
        let file = File::open(&path).map_err(|e| Error::io("open spill for mmap", e))?;
        let mmap = unsafe { Mmap::map(&file) }.map_err(|e| Error::io("mmap spill file", e))?;
        let expected = record_width
            .checked_mul(count)
            .ok_or_else(|| Error::InvalidInput("spill size overflow".into()))?;
        if mmap.len() != expected {
            return Err(Error::InvalidInput(format!(
                "spill file is {} bytes, expected {}",
                mmap.len(),
                expected
            )));
        }
        Ok(Self {
            path,
            _file: file,
            mmap,
            layout,
            record_width,
            count,
            bounds,
        })
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn layout(&self) -> StreamingLayout {
        self.layout
    }

    pub fn bounds(&self) -> Bounds {
        self.bounds
    }

    #[inline]
    fn record_bytes(&self, index: usize) -> &[u8] {
        let start = index * self.record_width;
        &self.mmap[start..start + self.record_width]
    }

    #[inline]
    pub fn xyz_at(&self, index: usize) -> (f64, f64, f64) {
        debug_assert!(index < self.count);
        let bytes = self.record_bytes(index);
        let x = f64::from_le_bytes(bytes[0..8].try_into().expect("spill x width"));
        let y = f64::from_le_bytes(bytes[8..16].try_into().expect("spill y width"));
        let z = f64::from_le_bytes(bytes[16..24].try_into().expect("spill z width"));
        (x, y, z)
    }

    pub fn record_at(&self, index: usize) -> Result<LasPointRecord> {
        if index >= self.count {
            return Err(Error::InvalidInput(format!(
                "spill index {index} out of range (len {})",
                self.count
            )));
        }
        deserialize_le(self.record_bytes(index), &self.layout)
            .map_err(|e| Error::InvalidData(format!("decode spill record {index}: {e}")))
    }
}

impl Drop for SpillReader {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layout_with_color() -> StreamingLayout {
        StreamingLayout {
            point_format: 3,
            has_gps: true,
            has_color: true,
            has_nir: false,
            has_waveform: false,
        }
    }

    fn record(seed: u32) -> LasPointRecord {
        let f = f64::from(seed);
        LasPointRecord {
            x: f * 1.5,
            y: -f * 2.25,
            z: f * 0.125,
            intensity: seed as u16,
            return_number: (seed % 5) as u8,
            number_of_returns: 5,
            classification: (seed % 32) as u8,
            scan_direction_flag: seed % 2 == 0,
            edge_of_flight_line: seed % 3 == 0,
            scan_angle: (seed as i16) - 100,
            user_data: (seed % 256) as u8,
            point_source_id: seed as u16,
            synthetic: seed % 4 == 0,
            key_point: seed % 4 == 1,
            withheld: seed % 4 == 2,
            overlap: false,
            scan_channel: 0,
            gps_time: 1.0e9 + f,
            red: (seed * 7) as u16,
            green: (seed * 11) as u16,
            blue: (seed * 13) as u16,
            nir: 0,
            wave_packet_descriptor_index: 0,
            byte_offset_to_waveform_data: 0,
            waveform_packet_size: 0,
            return_point_waveform_location: 0.0,
        }
    }

    #[test]
    fn spill_round_trips_records_and_bounds() {
        let dir = tempfile::tempdir().unwrap();
        let layout = layout_with_color();
        let mut writer = SpillWriter::create(dir.path(), layout).unwrap();
        let originals: Vec<LasPointRecord> = (0..256).map(record).collect();
        for rec in &originals {
            writer.push(rec).unwrap();
        }
        assert_eq!(writer.count(), 256);
        let reader = writer.finalize().unwrap();
        assert_eq!(reader.len(), 256);
        for (i, original) in originals.iter().enumerate() {
            assert_eq!(reader.record_at(i).unwrap(), *original);
            assert_eq!(reader.xyz_at(i), (original.x, original.y, original.z));
        }
        let bounds = reader.bounds();
        assert_eq!(bounds.min, (0.0, -573.75, 0.0));
        assert_eq!(bounds.max, (382.5, 0.0, 31.875));
    }

    #[test]
    fn unfinalized_spill_writer_removes_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = {
            let mut writer = SpillWriter::create(dir.path(), layout_with_color()).unwrap();
            writer.push(&record(1)).unwrap();
            writer.path.clone()
        };
        assert!(!path.exists());
    }

    #[test]
    fn finalized_spill_reader_removes_file() {
        let dir = tempfile::tempdir().unwrap();
        let mut writer = SpillWriter::create(dir.path(), layout_with_color()).unwrap();
        writer.push(&record(1)).unwrap();
        let reader = writer.finalize().unwrap();
        let path = reader.path.clone();
        assert!(path.exists());
        drop(reader);
        assert!(!path.exists());
    }
}
