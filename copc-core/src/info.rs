use crate::{Error, Result};

pub const COPC_INFO_BYTES: usize = 160;

/// COPC info VLR payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CopcInfo {
    pub center: (f64, f64, f64),
    pub halfsize: f64,
    pub spacing: f64,
    pub root_hier_offset: u64,
    pub root_hier_size: u64,
    pub gpstime_min: f64,
    pub gpstime_max: f64,
}

impl CopcInfo {
    pub fn from_le_bytes(bytes: &[u8]) -> Result<Self> {
        if bytes.len() < COPC_INFO_BYTES {
            return Err(Error::InvalidData(format!(
                "COPC info payload is {} bytes, expected at least {}",
                bytes.len(),
                COPC_INFO_BYTES
            )));
        }
        Ok(Self {
            center: (read_f64(bytes, 0), read_f64(bytes, 8), read_f64(bytes, 16)),
            halfsize: read_f64(bytes, 24),
            spacing: read_f64(bytes, 32),
            root_hier_offset: read_u64(bytes, 40),
            root_hier_size: read_u64(bytes, 48),
            gpstime_min: read_f64(bytes, 56),
            gpstime_max: read_f64(bytes, 64),
        })
    }

    pub fn write_le_bytes(self) -> [u8; COPC_INFO_BYTES] {
        let mut out = [0u8; COPC_INFO_BYTES];
        write_f64(&mut out, 0, self.center.0);
        write_f64(&mut out, 8, self.center.1);
        write_f64(&mut out, 16, self.center.2);
        write_f64(&mut out, 24, self.halfsize);
        write_f64(&mut out, 32, self.spacing);
        write_u64(&mut out, 40, self.root_hier_offset);
        write_u64(&mut out, 48, self.root_hier_size);
        write_f64(&mut out, 56, self.gpstime_min);
        write_f64(&mut out, 64, self.gpstime_max);
        out
    }
}

fn read_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("u64 width checked by caller"),
    )
}

fn read_f64(bytes: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("f64 width checked by caller"),
    )
}

fn write_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn write_f64(bytes: &mut [u8], offset: usize, value: f64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn copc_info_round_trips() {
        let info = CopcInfo {
            center: (1.0, 2.0, 3.0),
            halfsize: 4.0,
            spacing: 0.25,
            root_hier_offset: 100,
            root_hier_size: 320,
            gpstime_min: 10.0,
            gpstime_max: 20.0,
        };
        assert_eq!(
            CopcInfo::from_le_bytes(&info.write_le_bytes()).unwrap(),
            info
        );
    }
}
