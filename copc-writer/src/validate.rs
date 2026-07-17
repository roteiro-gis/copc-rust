//! Input validation and per-point statistics for COPC writes.

use copc_core::{Bounds, CancelCheck, Error, LasPointRecord, Result, StreamingLayout};

use crate::metadata::{has_wkt_crs_record, is_geotiff_crs_vlr, normalized_crs_wkt_override};
use crate::source::{CopcPointFields, CopcPointSource};
use crate::CANCEL_POLL_STRIDE;

const LAS_14_SCAN_ANGLE_SCALE: f32 = 0.006;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PointStats {
    pub(crate) gpstime_min: f64,
    pub(crate) gpstime_max: f64,
    pub(crate) extended_return_counts: [u64; 15],
}

impl PointStats {
    pub(crate) fn new() -> Self {
        Self {
            gpstime_min: f64::INFINITY,
            gpstime_max: f64::NEG_INFINITY,
            extended_return_counts: [0; 15],
        }
    }

    pub(crate) fn record(&mut self, index: usize, gps_time: f64, return_number: u8) -> Result<()> {
        if !gps_time.is_finite() {
            return Err(Error::InvalidInput(format!(
                "point {index} GPS time must be finite, got {gps_time}"
            )));
        }
        self.gpstime_min = self.gpstime_min.min(gps_time);
        self.gpstime_max = self.gpstime_max.max(gps_time);
        if (1..=15).contains(&return_number) {
            self.extended_return_counts[usize::from(return_number - 1)] += 1;
        }
        Ok(())
    }
}

pub(crate) fn validate_streaming_layout_supported(layout: &StreamingLayout) -> Result<()> {
    let mut unsupported = Vec::new();
    if layout.has_nir {
        unsupported.push("NIR point data".to_string());
    }
    if layout.has_waveform {
        unsupported.push("waveform point data".to_string());
    }
    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "COPC writer cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

pub(crate) fn validate_las_conversion_supported(
    header: &las::Header,
    source_evlrs: &[las::Vlr],
    crs_wkt_override: Option<&str>,
) -> Result<()> {
    let mut unsupported = Vec::new();
    let format = header.point_format();
    if format.has_nir {
        unsupported.push("NIR point data".to_string());
    }
    if format.has_waveform {
        unsupported.push("waveform point data".to_string());
    }
    let source_has_wkt_crs_record = has_wkt_crs_record(header, source_evlrs);
    let has_crs_wkt_override = normalized_crs_wkt_override(crs_wkt_override).is_some();
    let mut geotiff_crs_record_count = 0usize;
    for vlr in header.vlrs() {
        if is_geotiff_crs_vlr(vlr) && !source_has_wkt_crs_record && !has_crs_wkt_override {
            geotiff_crs_record_count += 1;
        }
    }
    for evlr in source_evlrs {
        if is_geotiff_crs_vlr(evlr) && !source_has_wkt_crs_record && !has_crs_wkt_override {
            geotiff_crs_record_count += 1;
        }
    }
    if geotiff_crs_record_count > 0 {
        unsupported.push(format!(
            "{geotiff_crs_record_count} GeoTIFF CRS VLR/EVLR(s); GeoTIFF-to-WKT CRS conversion is not implemented in copc-writer"
        ));
    }

    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "LAS-to-COPC streaming conversion cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

/// Validate the write-level inputs (bounds and LAS transform) shared by all
/// writer paths.
pub(crate) fn validate_write_setup(
    bounds: Bounds,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
) -> Result<()> {
    validate_bounds(bounds)?;
    validate_transform(scale, offset)
}

/// Validate one streamed record at spill intake: coordinates, scan angle, and
/// LAS field ranges. Spill-backed writes rely on this so the final write pass
/// needs no second full validation sweep.
pub(crate) fn validate_spill_record(record: &LasPointRecord, index: usize) -> Result<()> {
    validate_xyz_finite(index, record.x, record.y, record.z)?;
    validate_scan_angle(index, record.scan_angle)?;
    validate_point_field_range(index, "return_number", record.return_number, 0, 15)?;
    validate_point_field_range(index, "number_of_returns", record.number_of_returns, 0, 15)?;
    validate_point_field_range(index, "scan_channel", record.scan_channel, 0, 3)
}

/// Full per-point validation pass for sources that were not validated at
/// intake (for example `ColumnBatchSource`), accumulating output statistics.
pub(crate) fn validate_source_points<S: CopcPointSource>(
    source: &S,
    bounds: Bounds,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
    cancel: &dyn CancelCheck,
) -> Result<PointStats> {
    let extra_byte_count = usize::from(source.extra_byte_count());
    let mut stats = PointStats::new();
    let mut fields = CopcPointFields::default();
    for index in 0..source.len() {
        if index % CANCEL_POLL_STRIDE == 0 {
            cancel.check()?;
        }
        let (x, y, z) = source.xyz(index);
        validate_xyz_finite(index, x, y, z)?;
        validate_xyz_in_bounds(index, x, y, z, bounds)?;
        quantize_xyz(index, x, y, z, scale, offset)?;

        source.fields_into(index, &mut fields)?;
        validate_xyz_finite(index, fields.x, fields.y, fields.z)?;
        validate_xyz_in_bounds(index, fields.x, fields.y, fields.z, bounds)?;
        quantize_xyz(index, fields.x, fields.y, fields.z, scale, offset)?;
        validate_scan_angle(index, fields.scan_angle)?;
        validate_point_flags(index, &fields)?;
        if fields.extra_bytes.len() != extra_byte_count {
            return Err(Error::InvalidInput(format!(
                "point {index} has {} extra byte(s), expected {extra_byte_count}",
                fields.extra_bytes.len()
            )));
        }
        stats.record(index, fields.gps_time, fields.return_number)?;
    }
    Ok(stats)
}

fn validate_point_flags(index: usize, fields: &CopcPointFields) -> Result<()> {
    validate_point_field_range(index, "return_number", fields.return_number, 0, 15)?;
    validate_point_field_range(index, "number_of_returns", fields.number_of_returns, 0, 15)?;
    validate_point_field_range(index, "synthetic", fields.synthetic, 0, 1)?;
    validate_point_field_range(index, "key_point", fields.key_point, 0, 1)?;
    validate_point_field_range(index, "withheld", fields.withheld, 0, 1)?;
    validate_point_field_range(index, "overlap", fields.overlap, 0, 1)?;
    validate_point_field_range(index, "scan_channel", fields.scan_channel, 0, 3)?;
    validate_point_field_range(
        index,
        "scan_direction_flag",
        fields.scan_direction_flag,
        0,
        1,
    )?;
    validate_point_field_range(
        index,
        "edge_of_flight_line",
        fields.edge_of_flight_line,
        0,
        1,
    )
}

/// LAS 1.4 scaled scan angle from degrees, rounded to the nearest 0.006°
/// increment. (`las::raw::point::ScanAngle::from(f32)` truncates instead,
/// losing up to a full increment.) Float-to-int casts saturate, and inputs
/// are range-validated by `validate_scan_angle` before encoding.
pub(crate) fn scan_angle_to_las_scaled(value: f32) -> i16 {
    (value / LAS_14_SCAN_ANGLE_SCALE).round() as i16
}

fn validate_scan_angle(index: usize, value: f32) -> Result<()> {
    if !value.is_finite() {
        return Err(Error::InvalidInput(format!(
            "point {index} scan_angle must be finite, got {value}"
        )));
    }
    let scaled = value / LAS_14_SCAN_ANGLE_SCALE;
    if scaled < f32::from(i16::MIN) || scaled > f32::from(i16::MAX) {
        return Err(Error::InvalidInput(format!(
            "point {index} scan_angle {value} encodes to {scaled}, outside LAS i16 range"
        )));
    }
    Ok(())
}

fn validate_point_field_range(index: usize, name: &str, value: u8, min: u8, max: u8) -> Result<()> {
    if (min..=max).contains(&value) {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "point {index} {name} must be in {min}..={max}, got {value}"
        )))
    }
}

fn validate_bounds(bounds: Bounds) -> Result<()> {
    validate_finite_value("bounds min x", bounds.min.0)?;
    validate_finite_value("bounds min y", bounds.min.1)?;
    validate_finite_value("bounds min z", bounds.min.2)?;
    validate_finite_value("bounds max x", bounds.max.0)?;
    validate_finite_value("bounds max y", bounds.max.1)?;
    validate_finite_value("bounds max z", bounds.max.2)?;
    for (axis, min, max) in [
        ("x", bounds.min.0, bounds.max.0),
        ("y", bounds.min.1, bounds.max.1),
        ("z", bounds.min.2, bounds.max.2),
    ] {
        if min > max {
            return Err(Error::InvalidInput(format!(
                "bounds {axis} min {min} exceeds max {max}"
            )));
        }
        validate_finite_value(&format!("bounds {axis} span"), max - min)?;
    }
    Ok(())
}

fn validate_transform(scale: (f64, f64, f64), offset: (f64, f64, f64)) -> Result<()> {
    for (axis, value) in [("x", scale.0), ("y", scale.1), ("z", scale.2)] {
        if !value.is_finite() || value <= 0.0 {
            return Err(Error::InvalidInput(format!(
                "LAS {axis} scale must be finite and positive, got {value}"
            )));
        }
    }
    validate_finite_value("LAS x offset", offset.0)?;
    validate_finite_value("LAS y offset", offset.1)?;
    validate_finite_value("LAS z offset", offset.2)?;
    Ok(())
}

fn validate_xyz_finite(index: usize, x: f64, y: f64, z: f64) -> Result<()> {
    validate_point_axis_finite(index, "x", x)?;
    validate_point_axis_finite(index, "y", y)?;
    validate_point_axis_finite(index, "z", z)
}

/// Points outside the declared bounds would be assigned to octree voxels whose
/// bounds do not contain them, silently hiding them from readers' spatial
/// queries — reject them up front instead.
fn validate_xyz_in_bounds(index: usize, x: f64, y: f64, z: f64, bounds: Bounds) -> Result<()> {
    if bounds.contains_xyz(x, y, z) {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "point {index} ({x}, {y}, {z}) lies outside the declared bounds {bounds:?}"
        )))
    }
}

fn validate_point_axis_finite(index: usize, axis: &str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate must be finite, got {value}"
        )))
    }
}

fn validate_finite_value(name: &str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Error::InvalidInput(format!(
            "{name} must be finite, got {value}"
        )))
    }
}

pub(crate) fn quantize_xyz(
    index: usize,
    x: f64,
    y: f64,
    z: f64,
    scale: (f64, f64, f64),
    offset: (f64, f64, f64),
) -> Result<(i32, i32, i32)> {
    Ok((
        quantize_axis(index, "x", x, scale.0, offset.0)?,
        quantize_axis(index, "y", y, scale.1, offset.1)?,
        quantize_axis(index, "z", z, scale.2, offset.2)?,
    ))
}

fn quantize_axis(index: usize, axis: &str, value: f64, scale: f64, offset: f64) -> Result<i32> {
    let scaled = ((value - offset) / scale).round();
    if !scaled.is_finite() {
        return Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate cannot be encoded with scale {scale} and offset {offset}"
        )));
    }
    if scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
        return Err(Error::InvalidInput(format!(
            "point {index} {axis} coordinate {value} encodes to {scaled}, outside LAS i32 range"
        )));
    }
    Ok(scaled as i32)
}
