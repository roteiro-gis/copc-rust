//! Streaming LAS point record plus explicit little-endian spill bytes.

use std::io;

use las::{point::Format as LasFormat, Header as LasHeader, Vlr};

const LASF_SPEC_USER_ID: &str = "LASF_Spec";
const EXTRA_BYTES_RECORD_ID: u16 = 4;

/// In-memory representation of one full-fidelity LAS point.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LasPointRecord {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: u16,
    pub return_number: u8,
    pub number_of_returns: u8,
    pub classification: u8,
    pub scan_direction_flag: bool,
    pub edge_of_flight_line: bool,
    /// Scan angle in degrees.
    pub scan_angle: f32,
    pub user_data: u8,
    pub point_source_id: u16,
    pub synthetic: bool,
    pub key_point: bool,
    pub withheld: bool,
    pub overlap: bool,
    pub scan_channel: u8,
    pub gps_time: f64,
    pub red: u16,
    pub green: u16,
    pub blue: u16,
    pub nir: u16,
    pub wave_packet_descriptor_index: u8,
    pub byte_offset_to_waveform_data: u64,
    pub waveform_packet_size: u32,
    pub return_point_waveform_location: f32,
    pub extra_bytes: Vec<u8>,
}

impl LasPointRecord {
    /// Convert from the canonical `las::Point` shape used by the `las` crate.
    pub fn from_las_point(point: &las::Point) -> Self {
        let scan_direction_flag =
            matches!(point.scan_direction, las::point::ScanDirection::LeftToRight);
        let scan_angle = point.scan_angle;
        let (red, green, blue) = match point.color {
            Some(color) => (color.red, color.green, color.blue),
            None => (32_768, 32_768, 32_768),
        };
        let (
            wave_packet_descriptor_index,
            byte_offset_to_waveform_data,
            waveform_packet_size,
            return_point_waveform_location,
        ) = match point.waveform.as_ref() {
            Some(wf) => (
                wf.wave_packet_descriptor_index,
                wf.byte_offset_to_waveform_data,
                wf.waveform_packet_size_in_bytes,
                wf.return_point_waveform_location,
            ),
            None => (0, 0, 0, 0.0),
        };
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
            intensity: point.intensity,
            return_number: point.return_number,
            number_of_returns: point.number_of_returns,
            classification: u8::from(point.classification),
            scan_direction_flag,
            edge_of_flight_line: point.is_edge_of_flight_line,
            scan_angle,
            user_data: point.user_data,
            point_source_id: point.point_source_id,
            synthetic: point.is_synthetic,
            key_point: point.is_key_point,
            withheld: point.is_withheld,
            overlap: point.is_overlap,
            scan_channel: point.scanner_channel,
            gps_time: point.gps_time.unwrap_or(0.0),
            red,
            green,
            blue,
            nir: point.nir.unwrap_or(0),
            wave_packet_descriptor_index,
            byte_offset_to_waveform_data,
            waveform_packet_size,
            return_point_waveform_location,
            extra_bytes: point.extra_bytes.clone(),
        }
    }
}

/// Records which optional dimensions are present in a streaming pass.
#[derive(Clone, Debug, PartialEq)]
pub struct StreamingLayout {
    pub point_format: u8,
    pub has_gps: bool,
    pub has_color: bool,
    pub has_nir: bool,
    pub has_waveform: bool,
    pub extra_bytes: u16,
    pub extra_bytes_descriptors: Vec<Vlr>,
}

impl StreamingLayout {
    pub fn from_las_format(format: LasFormat) -> Self {
        Self {
            point_format: format.to_u8().unwrap_or(0),
            has_gps: format.has_gps_time,
            has_color: format.has_color,
            has_nir: format.has_nir,
            has_waveform: format.has_waveform,
            extra_bytes: format.extra_bytes,
            extra_bytes_descriptors: Vec::new(),
        }
    }

    pub fn from_las_header(header: &LasHeader) -> Self {
        let mut layout = Self::from_las_format(*header.point_format());
        if layout.extra_bytes > 0 {
            layout.extra_bytes_descriptors = header
                .vlrs()
                .iter()
                .filter(|vlr| is_extra_bytes_descriptor_vlr(vlr))
                .cloned()
                .collect();
        }
        layout
    }

    /// Compute the spill width in bytes per record.
    pub const fn record_width(&self) -> usize {
        let mut width = ALWAYS_BYTES;
        if self.has_gps {
            width += GPS_BYTES;
        }
        if self.has_color {
            width += COLOR_BYTES;
        }
        if self.has_nir {
            width += NIR_BYTES;
        }
        if self.has_waveform {
            width += WAVEFORM_BYTES;
        }
        width += self.extra_bytes as usize;
        width
    }

    pub const fn max_record_width() -> usize {
        ALWAYS_BYTES + GPS_BYTES + COLOR_BYTES + NIR_BYTES + WAVEFORM_BYTES + u16::MAX as usize
    }
}

const ALWAYS_BYTES: usize = 8 + 8 + 8 + 2 + 1 + 1 + 1 + 1 + 1 + 4 + 1 + 2 + 1 + 1 + 1 + 1 + 1;
const GPS_BYTES: usize = 8;
const COLOR_BYTES: usize = 6;
const NIR_BYTES: usize = 2;
const WAVEFORM_BYTES: usize = 1 + 8 + 4 + 4;

fn is_extra_bytes_descriptor_vlr(vlr: &Vlr) -> bool {
    vlr.user_id == LASF_SPEC_USER_ID && vlr.record_id == EXTRA_BYTES_RECORD_ID
}

/// Serialize one record into `dst` using the fixed little-endian spill format.
pub fn serialize_le(
    record: &LasPointRecord,
    layout: &StreamingLayout,
    dst: &mut [u8],
) -> io::Result<()> {
    if dst.len() != layout.record_width() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "destination is {} bytes, expected {}",
                dst.len(),
                layout.record_width()
            ),
        ));
    }
    let expected_extra_bytes = usize::from(layout.extra_bytes);
    if record.extra_bytes.len() != expected_extra_bytes {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "record has {} extra byte(s), expected {expected_extra_bytes}",
                record.extra_bytes.len()
            ),
        ));
    }
    let mut offset = 0;

    write_f64(&mut offset, dst, record.x);
    write_f64(&mut offset, dst, record.y);
    write_f64(&mut offset, dst, record.z);
    write_u16(&mut offset, dst, record.intensity);
    write_u8(&mut offset, dst, record.return_number);
    write_u8(&mut offset, dst, record.number_of_returns);
    write_u8(&mut offset, dst, record.classification);
    write_u8(&mut offset, dst, u8::from(record.scan_direction_flag));
    write_u8(&mut offset, dst, u8::from(record.edge_of_flight_line));
    write_f32(&mut offset, dst, record.scan_angle);
    write_u8(&mut offset, dst, record.user_data);
    write_u16(&mut offset, dst, record.point_source_id);
    write_u8(&mut offset, dst, u8::from(record.synthetic));
    write_u8(&mut offset, dst, u8::from(record.key_point));
    write_u8(&mut offset, dst, u8::from(record.withheld));
    write_u8(&mut offset, dst, u8::from(record.overlap));
    write_u8(&mut offset, dst, record.scan_channel);

    if layout.has_gps {
        write_f64(&mut offset, dst, record.gps_time);
    }
    if layout.has_color {
        write_u16(&mut offset, dst, record.red);
        write_u16(&mut offset, dst, record.green);
        write_u16(&mut offset, dst, record.blue);
    }
    if layout.has_nir {
        write_u16(&mut offset, dst, record.nir);
    }
    if layout.has_waveform {
        write_u8(&mut offset, dst, record.wave_packet_descriptor_index);
        write_u64(&mut offset, dst, record.byte_offset_to_waveform_data);
        write_u32(&mut offset, dst, record.waveform_packet_size);
        write_f32(&mut offset, dst, record.return_point_waveform_location);
    }
    dst[offset..offset + expected_extra_bytes].copy_from_slice(&record.extra_bytes);
    offset += expected_extra_bytes;
    debug_assert_eq!(offset, layout.record_width());
    Ok(())
}

/// Deserialize one record from the little-endian spill bytes.
pub fn deserialize_le(src: &[u8], layout: &StreamingLayout) -> io::Result<LasPointRecord> {
    let mut record = LasPointRecord::default();
    deserialize_le_into(src, layout, &mut record)?;
    Ok(record)
}

/// Deserialize one record into `out`, reusing its `extra_bytes` allocation.
pub fn deserialize_le_into(
    src: &[u8],
    layout: &StreamingLayout,
    out: &mut LasPointRecord,
) -> io::Result<()> {
    if src.len() != layout.record_width() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "record is {} bytes, expected {}",
                src.len(),
                layout.record_width()
            ),
        ));
    }
    let mut offset = 0;
    let x = read_f64(&mut offset, src);
    let y = read_f64(&mut offset, src);
    let z = read_f64(&mut offset, src);
    let intensity = read_u16(&mut offset, src);
    let return_number = read_u8(&mut offset, src);
    let number_of_returns = read_u8(&mut offset, src);
    let classification = read_u8(&mut offset, src);
    let scan_direction_flag = read_u8(&mut offset, src) != 0;
    let edge_of_flight_line = read_u8(&mut offset, src) != 0;
    let scan_angle = read_f32(&mut offset, src);
    let user_data = read_u8(&mut offset, src);
    let point_source_id = read_u16(&mut offset, src);
    let synthetic = read_u8(&mut offset, src) != 0;
    let key_point = read_u8(&mut offset, src) != 0;
    let withheld = read_u8(&mut offset, src) != 0;
    let overlap = read_u8(&mut offset, src) != 0;
    let scan_channel = read_u8(&mut offset, src);
    let gps_time = if layout.has_gps {
        read_f64(&mut offset, src)
    } else {
        0.0
    };
    let (red, green, blue) = if layout.has_color {
        (
            read_u16(&mut offset, src),
            read_u16(&mut offset, src),
            read_u16(&mut offset, src),
        )
    } else {
        (0, 0, 0)
    };
    let nir = if layout.has_nir {
        read_u16(&mut offset, src)
    } else {
        0
    };
    let (
        wave_packet_descriptor_index,
        byte_offset_to_waveform_data,
        waveform_packet_size,
        return_point_waveform_location,
    ) = if layout.has_waveform {
        (
            read_u8(&mut offset, src),
            read_u64(&mut offset, src),
            read_u32(&mut offset, src),
            read_f32(&mut offset, src),
        )
    } else {
        (0, 0, 0, 0.0)
    };
    out.extra_bytes.clear();
    if layout.extra_bytes > 0 {
        let end = offset + usize::from(layout.extra_bytes);
        out.extra_bytes.extend_from_slice(&src[offset..end]);
        offset = end;
    }
    debug_assert_eq!(offset, layout.record_width());
    out.x = x;
    out.y = y;
    out.z = z;
    out.intensity = intensity;
    out.return_number = return_number;
    out.number_of_returns = number_of_returns;
    out.classification = classification;
    out.scan_direction_flag = scan_direction_flag;
    out.edge_of_flight_line = edge_of_flight_line;
    out.scan_angle = scan_angle;
    out.user_data = user_data;
    out.point_source_id = point_source_id;
    out.synthetic = synthetic;
    out.key_point = key_point;
    out.withheld = withheld;
    out.overlap = overlap;
    out.scan_channel = scan_channel;
    out.gps_time = gps_time;
    out.red = red;
    out.green = green;
    out.blue = blue;
    out.nir = nir;
    out.wave_packet_descriptor_index = wave_packet_descriptor_index;
    out.byte_offset_to_waveform_data = byte_offset_to_waveform_data;
    out.waveform_packet_size = waveform_packet_size;
    out.return_point_waveform_location = return_point_waveform_location;
    Ok(())
}

#[inline]
fn write_u8(offset: &mut usize, dst: &mut [u8], value: u8) {
    dst[*offset] = value;
    *offset += 1;
}

#[inline]
fn write_u16(offset: &mut usize, dst: &mut [u8], value: u16) {
    dst[*offset..*offset + 2].copy_from_slice(&value.to_le_bytes());
    *offset += 2;
}

#[inline]
fn write_u32(offset: &mut usize, dst: &mut [u8], value: u32) {
    dst[*offset..*offset + 4].copy_from_slice(&value.to_le_bytes());
    *offset += 4;
}

#[inline]
fn write_u64(offset: &mut usize, dst: &mut [u8], value: u64) {
    dst[*offset..*offset + 8].copy_from_slice(&value.to_le_bytes());
    *offset += 8;
}

#[inline]
fn write_f32(offset: &mut usize, dst: &mut [u8], value: f32) {
    dst[*offset..*offset + 4].copy_from_slice(&value.to_le_bytes());
    *offset += 4;
}

#[inline]
fn write_f64(offset: &mut usize, dst: &mut [u8], value: f64) {
    dst[*offset..*offset + 8].copy_from_slice(&value.to_le_bytes());
    *offset += 8;
}

#[inline]
fn read_u8(offset: &mut usize, src: &[u8]) -> u8 {
    let value = src[*offset];
    *offset += 1;
    value
}

#[inline]
fn read_u16(offset: &mut usize, src: &[u8]) -> u16 {
    let value = u16::from_le_bytes(src[*offset..*offset + 2].try_into().expect("u16 width"));
    *offset += 2;
    value
}

#[inline]
fn read_u32(offset: &mut usize, src: &[u8]) -> u32 {
    let value = u32::from_le_bytes(src[*offset..*offset + 4].try_into().expect("u32 width"));
    *offset += 4;
    value
}

#[inline]
fn read_u64(offset: &mut usize, src: &[u8]) -> u64 {
    let value = u64::from_le_bytes(src[*offset..*offset + 8].try_into().expect("u64 width"));
    *offset += 8;
    value
}

#[inline]
fn read_f32(offset: &mut usize, src: &[u8]) -> f32 {
    let value = f32::from_le_bytes(src[*offset..*offset + 4].try_into().expect("f32 width"));
    *offset += 4;
    value
}

#[inline]
fn read_f64(offset: &mut usize, src: &[u8]) -> f64 {
    let value = f64::from_le_bytes(src[*offset..*offset + 8].try_into().expect("f64 width"));
    *offset += 8;
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_record() -> LasPointRecord {
        LasPointRecord {
            x: 1234.5678,
            y: -9876.54321,
            z: 100.0,
            intensity: 0xBEEF,
            return_number: 3,
            number_of_returns: 5,
            classification: 7,
            scan_direction_flag: true,
            edge_of_flight_line: true,
            scan_angle: -12.34,
            user_data: 0x42,
            point_source_id: 0xCAFE,
            synthetic: true,
            key_point: false,
            withheld: true,
            overlap: false,
            scan_channel: 2,
            gps_time: 1.234e9,
            red: 0xAAAA,
            green: 0x5555,
            blue: 0xF00F,
            nir: 0xCDCD,
            wave_packet_descriptor_index: 9,
            byte_offset_to_waveform_data: 0xDEADBEEF,
            waveform_packet_size: 0xABCD,
            return_point_waveform_location: -42.5,
            extra_bytes: vec![0xA0, 0xB1, 0xC2],
        }
    }

    #[test]
    fn round_trip_every_optional_combination() {
        let template = fixture_record();
        for has_gps in [false, true] {
            for has_color in [false, true] {
                for has_nir in [false, true] {
                    for has_waveform in [false, true] {
                        let layout = StreamingLayout {
                            point_format: 10,
                            has_gps,
                            has_color,
                            has_nir,
                            has_waveform,
                            extra_bytes: 3,
                            extra_bytes_descriptors: Vec::new(),
                        };
                        let mut record = template.clone();
                        if !layout.has_gps {
                            record.gps_time = 0.0;
                        }
                        if !layout.has_color {
                            record.red = 0;
                            record.green = 0;
                            record.blue = 0;
                        }
                        if !layout.has_nir {
                            record.nir = 0;
                        }
                        if !layout.has_waveform {
                            record.wave_packet_descriptor_index = 0;
                            record.byte_offset_to_waveform_data = 0;
                            record.waveform_packet_size = 0;
                            record.return_point_waveform_location = 0.0;
                        }
                        let mut bytes = vec![0u8; layout.record_width()];
                        serialize_le(&record, &layout, &mut bytes).unwrap();
                        assert_eq!(deserialize_le(&bytes, &layout).unwrap(), record);
                    }
                }
            }
        }
    }

    #[test]
    fn from_las_point_preserves_fractional_scan_angle_degrees() {
        let point = las::Point {
            scan_angle: 30.25,
            ..Default::default()
        };

        let record = LasPointRecord::from_las_point(&point);

        assert_eq!(30.25, record.scan_angle);
    }

    #[test]
    fn from_las_format_records_presence_flags() {
        let layout0 = StreamingLayout::from_las_format(LasFormat::new(0).unwrap());
        assert!(!layout0.has_gps);
        assert!(!layout0.has_color);
        assert!(!layout0.has_nir);
        assert!(!layout0.has_waveform);
        assert_eq!(0, layout0.extra_bytes);

        let layout3 = StreamingLayout::from_las_format(LasFormat::new(3).unwrap());
        assert!(layout3.has_gps);
        assert!(layout3.has_color);
        assert!(!layout3.has_nir);
        assert!(!layout3.has_waveform);
        assert_eq!(0, layout3.extra_bytes);

        let mut format10 = LasFormat::new(10).unwrap();
        format10.extra_bytes = 7;
        let layout10 = StreamingLayout::from_las_format(format10);
        assert!(layout10.has_gps);
        assert!(layout10.has_color);
        assert!(layout10.has_nir);
        assert!(layout10.has_waveform);
        assert_eq!(7, layout10.extra_bytes);
    }
}
