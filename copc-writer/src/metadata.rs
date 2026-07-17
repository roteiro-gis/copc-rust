//! Output LAS metadata carried through COPC writes, plus VLR/EVLR
//! classification and source-metadata extraction.

use std::fs::File;
use std::io::{Seek, SeekFrom};
use std::path::Path;

use copc_core::{Error, Result, MAX_EVLR_COUNT};
use las::raw;

use crate::las_out::LAS_VLR_HEADER_BYTES;

pub(crate) const LASZIP_VLR_USER_ID: &str = "laszip encoded";
pub(crate) const LASZIP_VLR_RECORD_ID: u16 = 22204;
const LASF_PROJECTION_USER_ID: &str = "LASF_Projection";
const WKT_CRS_RECORD_ID: u16 = 2112;
const GEOTIFF_GEO_KEY_DIRECTORY_RECORD_ID: u16 = 34735;
const GEOTIFF_DOUBLE_PARAMS_RECORD_ID: u16 = 34736;
const GEOTIFF_ASCII_PARAMS_RECORD_ID: u16 = 34737;
const LASF_SPEC_USER_ID: &str = "LASF_Spec";
const EXTRA_BYTES_RECORD_ID: u16 = 4;
pub(crate) const WKT_GLOBAL_ENCODING_BIT: u16 = 16;

/// Caller-supplied metadata for generated COPC output.
///
/// Defaults produce a conformant file: current UTC creation date, `copc-rust`
/// identifiers, millimeter scale, offsets derived from the write bounds, and
/// an empty WKT CRS record (LAS 1.4 requires the WKT global-encoding bit for
/// point formats 6 and 7).
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct CopcWriteMetadata {
    /// WKT CRS emitted as the `LASF_Projection`/2112 VLR (null-terminated on
    /// write). `None` emits an empty WKT record to keep the file conformant.
    pub wkt_crs: Option<String>,
    pub file_source_id: u16,
    pub guid: [u8; 16],
    /// Defaults to `copc-rust`.
    pub system_identifier: Option<String>,
    /// Defaults to `copc-writer`.
    pub generating_software: Option<String>,
    /// `(day_of_year, year)`; defaults to the current UTC date.
    pub creation_date: Option<(u16, u16)>,
    /// Sets LAS global-encoding bit 0 (GPS time is standard GPS time minus
    /// 1e9 rather than GPS week time).
    pub gps_standard_time: bool,
    /// LAS scale factors; defaults to `(0.001, 0.001, 0.001)`.
    pub scale: Option<(f64, f64, f64)>,
    /// LAS offsets; defaults to the minimum corner of the write bounds.
    pub offset: Option<(f64, f64, f64)>,
}

impl CopcWriteMetadata {
    pub(crate) fn to_output(&self) -> OutputLasMetadata {
        let mut out = OutputLasMetadata {
            file_source_id: self.file_source_id,
            guid: self.guid,
            global_encoding: u16::from(self.gps_standard_time),
            ..OutputLasMetadata::default()
        };
        if let Some(system_identifier) = &self.system_identifier {
            out.system_identifier.clone_from(system_identifier);
        }
        if let Some(generating_software) = &self.generating_software {
            out.generating_software.clone_from(generating_software);
        }
        let (creation_day_of_year, creation_year) =
            self.creation_date.unwrap_or_else(current_utc_date);
        out.creation_day_of_year = creation_day_of_year;
        out.creation_year = creation_year;
        if let Some(scale) = self.scale {
            out.scale = scale;
        }
        out.offset = self.offset;
        if let Some(crs_wkt) = normalized_crs_wkt_override(self.wkt_crs.as_deref()) {
            out.crs_records.push(wkt_crs_record(crs_wkt));
        }
        out.ensure_wkt_conformance();
        out
    }
}

#[derive(Clone, Debug)]
pub(crate) struct OutputCrsRecord {
    pub(crate) vlr: las::Vlr,
    pub(crate) is_extended: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct OutputLasMetadata {
    pub(crate) file_source_id: u16,
    pub(crate) global_encoding: u16,
    pub(crate) guid: [u8; 16],
    pub(crate) system_identifier: String,
    pub(crate) generating_software: String,
    pub(crate) creation_day_of_year: u16,
    pub(crate) creation_year: u16,
    pub(crate) scale: (f64, f64, f64),
    pub(crate) offset: Option<(f64, f64, f64)>,
    pub(crate) crs_records: Vec<OutputCrsRecord>,
    pub(crate) pass_through_vlrs: Vec<las::Vlr>,
    pub(crate) pass_through_evlrs: Vec<las::Vlr>,
}

impl Default for OutputLasMetadata {
    fn default() -> Self {
        Self {
            file_source_id: 0,
            global_encoding: 0,
            guid: [0; 16],
            system_identifier: "copc-rust".to_string(),
            generating_software: "copc-writer".to_string(),
            creation_day_of_year: 0,
            creation_year: 0,
            scale: (0.001, 0.001, 0.001),
            offset: None,
            crs_records: Vec::new(),
            pass_through_vlrs: Vec::new(),
            pass_through_evlrs: Vec::new(),
        }
    }
}

impl OutputLasMetadata {
    pub(crate) fn from_las_header(
        header: &las::Header,
        source_evlrs: &[las::Vlr],
        crs_wkt_override: Option<&str>,
    ) -> Self {
        let mut global_encoding = u16::from(header.gps_time_type());
        if header.has_synthetic_return_numbers() {
            global_encoding |= 8;
        }
        let mut crs_records = extract_source_wkt_crs_records(header, source_evlrs);
        if crs_records.is_empty() && has_geotiff_crs_record(header, source_evlrs) {
            if let Some(crs_wkt) = normalized_crs_wkt_override(crs_wkt_override) {
                crs_records.push(wkt_crs_record(crs_wkt));
            }
        }
        if !crs_records.is_empty() {
            global_encoding |= WKT_GLOBAL_ENCODING_BIT;
        }
        let pass_through_vlrs = extract_pass_through_vlrs(header);
        let pass_through_evlrs = extract_pass_through_evlrs(source_evlrs);
        let transforms = header.transforms();
        let (creation_day_of_year, creation_year) = header
            .date()
            .map(|date| {
                let year = date.format("%Y").to_string().parse().unwrap_or(0);
                let day = date.format("%j").to_string().parse().unwrap_or(0);
                (day, year)
            })
            .unwrap_or_else(current_utc_date);

        let mut metadata = Self {
            file_source_id: header.file_source_id(),
            global_encoding,
            guid: *header.guid().as_bytes(),
            system_identifier: header.system_identifier().to_string(),
            generating_software: header.generating_software().to_string(),
            creation_day_of_year,
            creation_year,
            scale: (transforms.x.scale, transforms.y.scale, transforms.z.scale),
            offset: Some((
                transforms.x.offset,
                transforms.y.offset,
                transforms.z.offset,
            )),
            crs_records,
            pass_through_vlrs,
            pass_through_evlrs,
        };
        metadata.ensure_wkt_conformance();
        metadata
    }

    /// LAS 1.4 requires the WKT global-encoding bit for point formats 6-10;
    /// when no CRS record is carried, back the bit with an empty WKT CRS VLR
    /// (matching PDAL's behavior for CRS-less COPC output).
    fn ensure_wkt_conformance(&mut self) {
        self.global_encoding |= WKT_GLOBAL_ENCODING_BIT;
        if self.crs_records.is_empty() {
            self.crs_records.push(wkt_crs_record(""));
        }
    }

    pub(crate) fn regular_crs_vlrs(&self) -> impl Iterator<Item = &las::Vlr> {
        self.crs_records
            .iter()
            .filter(|record| !record.is_extended)
            .map(|record| &record.vlr)
    }

    pub(crate) fn extended_crs_evlrs(&self) -> impl Iterator<Item = &las::Vlr> {
        self.crs_records
            .iter()
            .filter(|record| record.is_extended)
            .map(|record| &record.vlr)
    }

    pub(crate) fn regular_crs_vlr_count(&self) -> usize {
        self.crs_records
            .iter()
            .filter(|record| !record.is_extended)
            .count()
    }

    pub(crate) fn extended_crs_evlr_count(&self) -> usize {
        self.crs_records
            .iter()
            .filter(|record| record.is_extended)
            .count()
    }

    pub(crate) fn regular_crs_vlr_bytes(&self) -> Result<u32> {
        self.regular_crs_vlrs().try_fold(0u32, |total, vlr| {
            let data_len = u16::try_from(vlr.data.len()).map_err(|_| {
                Error::InvalidInput(format!(
                    "regular WKT CRS VLR is too large: {} byte(s)",
                    vlr.data.len()
                ))
            })?;
            total
                .checked_add(LAS_VLR_HEADER_BYTES + u32::from(data_len))
                .ok_or_else(|| Error::InvalidInput("CRS VLR byte size overflow".into()))
        })
    }

    pub(crate) fn source_evlrs_after_hierarchy(&self) -> impl Iterator<Item = &las::Vlr> {
        self.extended_crs_evlrs()
            .chain(self.pass_through_evlrs.iter())
    }

    pub(crate) fn source_evlr_count_after_hierarchy(&self) -> usize {
        self.extended_crs_evlr_count() + self.pass_through_evlrs.len()
    }
}

pub(crate) fn read_all_source_evlrs(path: &Path) -> Result<Vec<las::Vlr>> {
    let mut file = File::open(path).map_err(|e| Error::io("open source LAS/LAZ", e))?;
    let raw_header =
        raw::Header::read_from(&mut file).map_err(|e| Error::Las(format!("source header: {e}")))?;
    let Some(evlr_header) = raw_header.evlr else {
        return Ok(Vec::new());
    };

    file.seek(SeekFrom::Start(evlr_header.start_of_first_evlr))
        .map_err(|e| Error::io("seek source EVLRs", e))?;
    if evlr_header.number_of_evlrs > MAX_EVLR_COUNT {
        return Err(Error::InvalidData(format!(
            "source EVLR count {} exceeds max supported {MAX_EVLR_COUNT}",
            evlr_header.number_of_evlrs
        )));
    }
    let evlr_count = usize::try_from(evlr_header.number_of_evlrs)
        .map_err(|_| Error::InvalidInput("source EVLR count overflows usize".into()))?;
    let mut evlrs = Vec::with_capacity(evlr_count);
    for index in 0..evlr_header.number_of_evlrs {
        let evlr = raw::Vlr::read_from(&mut file, true)
            .map(las::Vlr::new)
            .map_err(|e| Error::Las(format!("source EVLR {index}: {e}")))?;
        evlrs.push(evlr);
    }
    Ok(evlrs)
}

fn is_laszip_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASZIP_VLR_USER_ID && vlr.record_id == LASZIP_VLR_RECORD_ID
}

fn is_copc_info_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == "copc" && vlr.record_id == 1
}

fn is_copc_hierarchy_evlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == "copc" && vlr.record_id == 1000
}

fn is_wkt_crs_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASF_PROJECTION_USER_ID && vlr.record_id == WKT_CRS_RECORD_ID
}

pub(crate) fn is_geotiff_crs_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASF_PROJECTION_USER_ID
        && matches!(
            vlr.record_id,
            GEOTIFF_GEO_KEY_DIRECTORY_RECORD_ID
                | GEOTIFF_DOUBLE_PARAMS_RECORD_ID
                | GEOTIFF_ASCII_PARAMS_RECORD_ID
        )
}

pub(crate) fn normalized_crs_wkt_override(crs_wkt_override: Option<&str>) -> Option<&str> {
    crs_wkt_override.filter(|crs_wkt| !crs_wkt.trim().is_empty())
}

fn is_extra_bytes_descriptor_vlr(vlr: &las::Vlr) -> bool {
    vlr.user_id == LASF_SPEC_USER_ID && vlr.record_id == EXTRA_BYTES_RECORD_ID
}

pub(crate) fn has_wkt_crs_record(header: &las::Header, source_evlrs: &[las::Vlr]) -> bool {
    header.vlrs().iter().any(is_wkt_crs_vlr) || source_evlrs.iter().any(is_wkt_crs_vlr)
}

fn has_geotiff_crs_record(header: &las::Header, source_evlrs: &[las::Vlr]) -> bool {
    header.vlrs().iter().any(is_geotiff_crs_vlr) || source_evlrs.iter().any(is_geotiff_crs_vlr)
}

fn extract_source_wkt_crs_records(
    header: &las::Header,
    source_evlrs: &[las::Vlr],
) -> Vec<OutputCrsRecord> {
    let mut records = Vec::new();
    for vlr in header.vlrs() {
        if is_wkt_crs_vlr(vlr) {
            records.push(OutputCrsRecord {
                vlr: vlr.clone(),
                is_extended: false,
            });
        }
    }
    for evlr in source_evlrs {
        if is_wkt_crs_vlr(evlr) {
            records.push(OutputCrsRecord {
                vlr: evlr.clone(),
                is_extended: true,
            });
        }
    }
    records
}

fn wkt_crs_record(crs_wkt: &str) -> OutputCrsRecord {
    OutputCrsRecord {
        vlr: las::Vlr {
            user_id: LASF_PROJECTION_USER_ID.to_string(),
            record_id: WKT_CRS_RECORD_ID,
            description: "OGC WKT CRS".to_string(),
            data: null_terminated_wkt_bytes(crs_wkt),
        },
        is_extended: false,
    }
}

fn null_terminated_wkt_bytes(crs_wkt: &str) -> Vec<u8> {
    let mut data = crs_wkt.as_bytes().to_vec();
    if !data.ends_with(&[0]) {
        data.push(0);
    }
    data
}

/// Current UTC date as LAS `(day_of_year, year)`.
fn current_utc_date() -> (u16, u16) {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|elapsed| elapsed.as_secs())
        .unwrap_or(0);
    utc_date_from_unix_days((secs / 86_400) as i64)
}

/// Civil-from-days (Howard Hinnant's algorithm) reduced to `(day_of_year, year)`.
fn utc_date_from_unix_days(days_since_epoch: i64) -> (u16, u16) {
    let z = days_since_epoch + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy_from_march = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy_from_march + 2) / 153;
    let day = (doy_from_march - (153 * mp + 2) / 5 + 1) as u16;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u8;
    let year = if month <= 2 { y + 1 } else { y };

    const CUMULATIVE_DAYS: [u16; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    let mut day_of_year = CUMULATIVE_DAYS[usize::from(month - 1)] + day;
    if leap && month > 2 {
        day_of_year += 1;
    }
    (day_of_year, year as u16)
}

fn extract_pass_through_vlrs(header: &las::Header) -> Vec<las::Vlr> {
    header
        .vlrs()
        .iter()
        .filter(|vlr| !is_laszip_vlr(vlr))
        .filter(|vlr| !is_copc_info_vlr(vlr))
        .filter(|vlr| !is_copc_hierarchy_evlr(vlr))
        .filter(|vlr| !is_wkt_crs_vlr(vlr))
        .filter(|vlr| !is_geotiff_crs_vlr(vlr))
        .filter(|vlr| !is_extra_bytes_descriptor_vlr(vlr))
        .cloned()
        .collect()
}

fn extract_pass_through_evlrs(source_evlrs: &[las::Vlr]) -> Vec<las::Vlr> {
    source_evlrs
        .iter()
        .filter(|evlr| !is_laszip_vlr(evlr))
        .filter(|evlr| !is_copc_info_vlr(evlr))
        .filter(|evlr| !is_copc_hierarchy_evlr(evlr))
        .filter(|evlr| !is_wkt_crs_vlr(evlr))
        .filter(|evlr| !is_geotiff_crs_vlr(evlr))
        .cloned()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_date_from_unix_days_matches_known_dates() {
        // 1970-01-01
        assert_eq!((1, 1970), utc_date_from_unix_days(0));
        // 2000-12-31 (leap year, day 366) = 11322 days after epoch
        assert_eq!((366, 2000), utc_date_from_unix_days(11_322));
        // 2024-03-01 (leap year) = 19783 days after epoch
        assert_eq!((61, 2024), utc_date_from_unix_days(19_783));
        // 2026-07-10 = 20644 days after epoch
        assert_eq!((191, 2026), utc_date_from_unix_days(20_644));
    }

    #[test]
    fn write_metadata_defaults_are_wkt_conformant() {
        let output = CopcWriteMetadata::default().to_output();

        assert_ne!(0, output.global_encoding & WKT_GLOBAL_ENCODING_BIT);
        assert_eq!(1, output.regular_crs_vlr_count());
        let wkt_vlr = output.regular_crs_vlrs().next().unwrap();
        assert_eq!(vec![0u8], wkt_vlr.data);
        assert!(output.creation_year >= 2026);
        assert!((1..=366).contains(&output.creation_day_of_year));
    }

    #[test]
    fn write_metadata_carries_caller_wkt_and_date() {
        let metadata = CopcWriteMetadata {
            wkt_crs: Some("PROJCS[\"test\"]".to_string()),
            creation_date: Some((42, 2001)),
            gps_standard_time: true,
            ..CopcWriteMetadata::default()
        };

        let output = metadata.to_output();

        assert_eq!(
            (42, 2001),
            (output.creation_day_of_year, output.creation_year)
        );
        assert_eq!(1, output.global_encoding & 1);
        let wkt_vlr = output.regular_crs_vlrs().next().unwrap();
        assert_eq!(b"PROJCS[\"test\"]\0".as_slice(), wkt_vlr.data.as_slice());
    }
}
