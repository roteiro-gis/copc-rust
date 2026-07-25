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
/// (which forwards to `ureq/rustls`) for HTTPS URLs.
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
            agent: ureq::Agent::new_with_defaults(),
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
            .header("Range", "bytes=0-0")
            .header("Accept-Encoding", "identity")
            .call()
            .map_err(|e| Error::InvalidInput(format!("GET {}: {e}", self.url)))?;
        if response.status() != 206 {
            return Err(Error::Unsupported(format!(
                "GET {} returned status {}; the server must support HTTP range requests",
                self.url,
                response.status()
            )));
        }
        let (start, end, len) = response
            .headers()
            .get("content-range")
            .and_then(|value| value.to_str().ok())
            .and_then(parse_content_range)
            .ok_or_else(|| {
                Error::InvalidData(format!(
                    "GET {} returned no usable Content-Range header",
                    self.url
                ))
            })?;
        if (start, end) != (0, 0) {
            return Err(Error::InvalidData(format!(
                "GET {} returned Content-Range bytes {start}-{end}, expected bytes 0-0",
                self.url
            )));
        }
        if len == 0 {
            return Err(Error::InvalidData(format!(
                "GET {} returned a zero Content-Range total",
                self.url
            )));
        }
        let mut reader = response.into_body().into_reader();
        let mut probe = [0u8; 1];
        reader
            .read_exact(&mut probe)
            .map_err(|e| Error::io("read HTTP length probe body", e))?;
        let mut trailing = [0u8; 1];
        if reader
            .read(&mut trailing)
            .map_err(|e| Error::io("check HTTP length probe body", e))?
            != 0
        {
            return Err(Error::InvalidData(format!(
                "GET {} returned more bytes than its Content-Range",
                self.url
            )));
        }
        self.len = Some(len);
        Ok(len)
    }

    fn read_range(&mut self, offset: u64, buf: &mut [u8]) -> Result<()> {
        if buf.is_empty() {
            return Ok(());
        }
        let byte_len = u64::try_from(buf.len())
            .map_err(|_| Error::InvalidInput("HTTP range length exceeds u64".into()))?;
        let end_inclusive = offset
            .checked_add(byte_len - 1)
            .ok_or_else(|| Error::InvalidInput("HTTP range end overflows u64".into()))?;
        let response = self
            .agent
            .get(&self.url)
            .header("Range", format!("bytes={offset}-{end_inclusive}"))
            .header("Accept-Encoding", "identity")
            .call()
            .map_err(|e| Error::InvalidInput(format!("GET {}: {e}", self.url)))?;
        if response.status() != 206 {
            return Err(Error::Unsupported(format!(
                "GET {} returned status {}; the server must support HTTP range requests",
                self.url,
                response.status()
            )));
        }
        let (response_start, response_end, response_len) = response
            .headers()
            .get("content-range")
            .and_then(|value| value.to_str().ok())
            .and_then(parse_content_range)
            .ok_or_else(|| {
                Error::InvalidData(format!(
                    "GET {} returned no usable Content-Range header",
                    self.url
                ))
            })?;
        if (response_start, response_end) != (offset, end_inclusive) {
            return Err(Error::InvalidData(format!(
                "GET {} returned Content-Range bytes {response_start}-{response_end}, expected bytes {offset}-{end_inclusive}",
                self.url
            )));
        }
        if let Some(expected_len) = self.len {
            if response_len != expected_len {
                return Err(Error::InvalidData(format!(
                    "GET {} Content-Range total changed from {expected_len} to {response_len}",
                    self.url
                )));
            }
        } else {
            self.len = Some(response_len);
        }
        let mut reader = response.into_body().into_reader();
        reader
            .read_exact(buf)
            .map_err(|e| Error::io("read HTTP range body", e))?;
        let mut trailing = [0u8; 1];
        if reader
            .read(&mut trailing)
            .map_err(|e| Error::io("check HTTP range body length", e))?
            != 0
        {
            return Err(Error::InvalidData(format!(
                "GET {} returned more bytes than its Content-Range",
                self.url
            )));
        }
        Ok(())
    }
}

#[cfg(feature = "http")]
fn parse_content_range(value: &str) -> Option<(u64, u64, u64)> {
    let value = value.strip_prefix("bytes ")?;
    let (range, total) = value.split_once('/')?;
    let (start, end) = range.split_once('-')?;
    let start = start.parse().ok()?;
    let end = end.parse().ok()?;
    let total = total.parse().ok()?;
    (start <= end && end < total).then_some((start, end, total))
}

#[cfg(all(test, feature = "http"))]
mod tests {
    use super::*;

    #[test]
    fn parses_strict_content_ranges() {
        assert_eq!(Some((0, 0, 42)), parse_content_range("bytes 0-0/42"));
        assert_eq!(None, parse_content_range("bytes 1-0/42"));
        assert_eq!(None, parse_content_range("bytes 0-42/42"));
        assert_eq!(None, parse_content_range("0-0/42"));
        assert_eq!(None, parse_content_range("bytes 0-0/*"));
    }
}
