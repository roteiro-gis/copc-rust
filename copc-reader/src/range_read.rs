//! Byte-range sources for remote and partial COPC reads.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};

use copc_core::{Error, Result};

/// A random-access byte source addressed by absolute offsets.
///
/// Implementations back [`crate::CopcRangeReader`], which fetches only the
/// header, the hierarchy pages a query needs, and the selected point chunks.
#[allow(clippy::len_without_is_empty)]
pub trait RangeRead {
    /// Total length of the source in bytes.
    fn len(&mut self) -> Result<u64>;

    /// Fill `buf` with the bytes at `offset..offset + buf.len()`.
    fn read_range(&mut self, offset: u64, buf: &mut [u8]) -> Result<()>;
}

impl RangeRead for File {
    fn len(&mut self) -> Result<u64> {
        Ok(self
            .metadata()
            .map_err(|e| Error::io("stat COPC file", e))?
            .len())
    }

    fn read_range(&mut self, offset: u64, buf: &mut [u8]) -> Result<()> {
        self.seek(SeekFrom::Start(offset))
            .map_err(|e| Error::io("seek COPC range", e))?;
        self.read_exact(buf)
            .map_err(|e| Error::io("read COPC range", e))
    }
}

/// HTTP(S) byte-range source using `Range` requests.
///
/// Plain HTTP works out of the box; enable the crate's `http-tls` feature
/// (which forwards to `ureq/tls`) for HTTPS URLs.
#[cfg(feature = "http")]
pub struct HttpRangeReader {
    agent: ureq::Agent,
    url: String,
    len: Option<u64>,
}

#[cfg(feature = "http")]
impl HttpRangeReader {
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            agent: ureq::Agent::new(),
            url: url.into(),
            len: None,
        }
    }
}

#[cfg(feature = "http")]
impl RangeRead for HttpRangeReader {
    /// Probes the total length with a one-byte range request, which also
    /// verifies up front that the server honors `Range` headers.
    fn len(&mut self) -> Result<u64> {
        if let Some(len) = self.len {
            return Ok(len);
        }
        let response = self
            .agent
            .get(&self.url)
            .set("Range", "bytes=0-0")
            .call()
            .map_err(|e| Error::InvalidInput(format!("GET {}: {e}", self.url)))?;
        if response.status() != 206 {
            return Err(Error::Unsupported(format!(
                "GET {} returned status {}; the server must support HTTP range requests",
                self.url,
                response.status()
            )));
        }
        // Content-Range: bytes 0-0/<total>
        let len = response
            .header("content-range")
            .and_then(|value| value.rsplit('/').next())
            .and_then(|total| total.parse::<u64>().ok())
            .ok_or_else(|| {
                Error::InvalidData(format!(
                    "GET {} returned no usable content-range total",
                    self.url
                ))
            })?;
        self.len = Some(len);
        Ok(len)
    }

    fn read_range(&mut self, offset: u64, buf: &mut [u8]) -> Result<()> {
        if buf.is_empty() {
            return Ok(());
        }
        let end_inclusive = offset
            .checked_add(buf.len() as u64 - 1)
            .ok_or_else(|| Error::InvalidInput("HTTP range end overflows u64".into()))?;
        let response = self
            .agent
            .get(&self.url)
            .set("Range", &format!("bytes={offset}-{end_inclusive}"))
            .call()
            .map_err(|e| Error::InvalidInput(format!("GET {}: {e}", self.url)))?;
        if response.status() != 206 {
            return Err(Error::Unsupported(format!(
                "GET {} returned status {}; the server must support HTTP range requests",
                self.url,
                response.status()
            )));
        }
        response
            .into_reader()
            .read_exact(buf)
            .map_err(|e| Error::io("read HTTP range body", e))
    }
}
