//! Point-data sources consumed by the COPC writer.

use std::cell::RefCell;

use copc_core::{Bounds, ColumnData, Error, LasColumnBatch, LasDimension, LasPointRecord, Result};

use crate::spill::SpillReader;

/// Normalized point fields consumed by the COPC writer.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CopcPointFields {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: u16,
    pub return_number: u8,
    pub number_of_returns: u8,
    pub synthetic: u8,
    pub key_point: u8,
    pub withheld: u8,
    pub overlap: u8,
    pub scan_channel: u8,
    pub scan_direction_flag: u8,
    pub edge_of_flight_line: u8,
    pub classification: u8,
    pub user_data: u8,
    /// Scan angle in degrees; encoded as LAS 1.4 scaled scan angle on write.
    pub scan_angle: f32,
    pub point_source_id: u16,
    pub gps_time: f64,
    pub red: u16,
    pub green: u16,
    pub blue: u16,
    pub extra_bytes: Vec<u8>,
}

/// Abstract point-data source for COPC emission.
pub trait CopcPointSource {
    fn len(&self) -> usize;
    /// Return the coordinates used for octree assignment.
    fn xyz(&self, index: usize) -> Result<(f64, f64, f64)>;
    /// Fill `out` with the fields of the point at `index`, reusing `out`'s
    /// allocations (notably `extra_bytes`) across calls.
    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> Result<()>;
    fn extra_byte_count(&self) -> u16 {
        0
    }
    fn extra_bytes_vlrs(&self) -> &[las::Vlr] {
        &[]
    }

    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// COPC writer source backed directly by a neutral LAS column batch.
pub struct ColumnBatchSource<'a> {
    batch: &'a LasColumnBatch,
    x: &'a [f64],
    y: &'a [f64],
    z: &'a [f64],
    intensity: Option<&'a [u16]>,
    return_number: Option<&'a [u8]>,
    number_of_returns: Option<&'a [u8]>,
    synthetic: Option<&'a [bool]>,
    key_point: Option<&'a [bool]>,
    withheld: Option<&'a [bool]>,
    overlap: Option<&'a [bool]>,
    scan_channel: Option<&'a [u8]>,
    scan_direction_flag: Option<&'a [bool]>,
    edge_of_flight_line: Option<&'a [bool]>,
    classification: Option<&'a [u8]>,
    user_data: Option<&'a [u8]>,
    scan_angle: Option<&'a [f32]>,
    point_source_id: Option<&'a [u16]>,
    gps_time: Option<&'a [f64]>,
    red: Option<&'a [u16]>,
    green: Option<&'a [u16]>,
    blue: Option<&'a [u16]>,
    extra_bytes: Option<(&'a [u8], usize)>,
}

impl<'a> ColumnBatchSource<'a> {
    pub fn new(batch: &'a LasColumnBatch) -> Result<Self> {
        batch.validate()?;
        validate_column_batch_writer_support(batch)?;

        let x = required_f64_column(batch, LasDimension::X)?;
        let y = required_f64_column(batch, LasDimension::Y)?;
        let z = required_f64_column(batch, LasDimension::Z)?;
        let red = optional_u16_column(batch, LasDimension::Red)?;
        let green = optional_u16_column(batch, LasDimension::Green)?;
        let blue = optional_u16_column(batch, LasDimension::Blue)?;
        let extra_bytes = optional_extra_bytes_column(batch)?;
        validate_color_columns(red, green, blue)?;

        Ok(Self {
            batch,
            x,
            y,
            z,
            intensity: optional_u16_column(batch, LasDimension::Intensity)?,
            return_number: optional_u8_column(batch, LasDimension::ReturnNumber)?,
            number_of_returns: optional_u8_column(batch, LasDimension::NumberOfReturns)?,
            synthetic: optional_bool_column(batch, LasDimension::Synthetic)?,
            key_point: optional_bool_column(batch, LasDimension::KeyPoint)?,
            withheld: optional_bool_column(batch, LasDimension::Withheld)?,
            overlap: optional_bool_column(batch, LasDimension::Overlap)?,
            scan_channel: optional_u8_column(batch, LasDimension::ScanChannel)?,
            scan_direction_flag: optional_bool_column(batch, LasDimension::ScanDirectionFlag)?,
            edge_of_flight_line: optional_bool_column(batch, LasDimension::EdgeOfFlightLine)?,
            classification: optional_u8_column(batch, LasDimension::Classification)?,
            user_data: optional_u8_column(batch, LasDimension::UserData)?,
            scan_angle: optional_f32_column(batch, LasDimension::ScanAngle)?,
            point_source_id: optional_u16_column(batch, LasDimension::PointSourceId)?,
            gps_time: optional_f64_column(batch, LasDimension::GpsTime)?,
            red,
            green,
            blue,
            extra_bytes,
        })
    }

    pub fn batch(&self) -> &LasColumnBatch {
        self.batch
    }

    pub fn has_color(&self) -> bool {
        self.red.is_some() && self.green.is_some() && self.blue.is_some()
    }

    pub fn extra_byte_width(&self) -> usize {
        self.extra_bytes.map(|(_, width)| width).unwrap_or(0)
    }

    pub fn bounds(&self) -> Result<Bounds> {
        if self.is_empty() {
            return Err(Error::InvalidInput(
                "cannot compute bounds for empty column batch".into(),
            ));
        }
        let mut bounds = Bounds::point(self.x[0], self.y[0], self.z[0]);
        for index in 1..self.len() {
            bounds.extend(self.x[index], self.y[index], self.z[index]);
        }
        Ok(bounds)
    }
}

impl CopcPointSource for ColumnBatchSource<'_> {
    fn len(&self) -> usize {
        self.batch.len()
    }

    #[inline]
    fn xyz(&self, index: usize) -> Result<(f64, f64, f64)> {
        if index >= self.len() {
            return Err(Error::InvalidInput(format!(
                "column batch index {index} out of range (len {})",
                self.len()
            )));
        }
        Ok((self.x[index], self.y[index], self.z[index]))
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> Result<()> {
        out.x = self.x[index];
        out.y = self.y[index];
        out.z = self.z[index];
        out.intensity = at_u16(self.intensity, index);
        out.return_number = at_u8(self.return_number, index);
        out.number_of_returns = at_u8(self.number_of_returns, index);
        out.synthetic = at_bool_u8(self.synthetic, index);
        out.key_point = at_bool_u8(self.key_point, index);
        out.withheld = at_bool_u8(self.withheld, index);
        out.overlap = at_bool_u8(self.overlap, index);
        out.scan_channel = at_u8(self.scan_channel, index);
        out.scan_direction_flag = at_bool_u8(self.scan_direction_flag, index);
        out.edge_of_flight_line = at_bool_u8(self.edge_of_flight_line, index);
        out.classification = at_u8(self.classification, index);
        out.user_data = at_u8(self.user_data, index);
        out.scan_angle = self.scan_angle.map(|column| column[index]).unwrap_or(0.0);
        out.point_source_id = at_u16(self.point_source_id, index);
        out.gps_time = self.gps_time.map(|column| column[index]).unwrap_or(0.0);
        out.red = at_u16(self.red, index);
        out.green = at_u16(self.green, index);
        out.blue = at_u16(self.blue, index);
        out.extra_bytes.clear();
        if let Some((values, width)) = self.extra_bytes {
            let start = index * width;
            out.extra_bytes
                .extend_from_slice(&values[start..start + width]);
        }
        Ok(())
    }

    fn extra_byte_count(&self) -> u16 {
        self.extra_byte_width() as u16
    }
}

fn at_bool_u8(column: Option<&[bool]>, index: usize) -> u8 {
    column.map(|values| u8::from(values[index])).unwrap_or(0)
}

fn at_u8(column: Option<&[u8]>, index: usize) -> u8 {
    column.map(|values| values[index]).unwrap_or(0)
}

fn at_u16(column: Option<&[u16]>, index: usize) -> u16 {
    column.map(|values| values[index]).unwrap_or(0)
}

fn validate_column_batch_writer_support(batch: &LasColumnBatch) -> Result<()> {
    let unsupported: Vec<_> = batch
        .columns
        .iter()
        .filter_map(|(spec, _)| match spec.dimension {
            LasDimension::Nir => Some("NIR point data"),
            LasDimension::WaveformPacketDescriptorIndex
            | LasDimension::WaveformPacketByteOffset
            | LasDimension::WaveformPacketSize
            | LasDimension::WavePacketReturnPointWaveformLocation => Some("waveform point data"),
            _ => None,
        })
        .collect();
    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(Error::Unsupported(format!(
            "COPC writer cannot preserve {}",
            unsupported.join(", ")
        )))
    }
}

fn validate_color_columns(
    red: Option<&[u16]>,
    green: Option<&[u16]>,
    blue: Option<&[u16]>,
) -> Result<()> {
    let present =
        usize::from(red.is_some()) + usize::from(green.is_some()) + usize::from(blue.is_some());
    if present == 0 || present == 3 {
        Ok(())
    } else {
        Err(Error::InvalidInput(
            "Red, Green, and Blue columns must be supplied together".into(),
        ))
    }
}

fn required_f64_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<&[f64]> {
    match batch.column(dimension) {
        Some(ColumnData::F64(values)) => Ok(values),
        Some(other) => Err(unexpected_column_type(dimension, "F64", other)),
        None => Err(Error::InvalidInput(format!(
            "ColumnBatchSource requires {dimension:?} column"
        ))),
    }
}

fn optional_f64_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[f64]>> {
    match batch.column(dimension) {
        Some(ColumnData::F64(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "F64", other)),
        None => Ok(None),
    }
}

fn optional_f32_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[f32]>> {
    match batch.column(dimension) {
        Some(ColumnData::F32(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "F32", other)),
        None => Ok(None),
    }
}

fn optional_u16_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[u16]>> {
    match batch.column(dimension) {
        Some(ColumnData::U16(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "U16", other)),
        None => Ok(None),
    }
}

fn optional_u8_column(batch: &LasColumnBatch, dimension: LasDimension) -> Result<Option<&[u8]>> {
    match batch.column(dimension) {
        Some(ColumnData::U8(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "U8", other)),
        None => Ok(None),
    }
}

fn optional_extra_bytes_column(batch: &LasColumnBatch) -> Result<Option<(&[u8], usize)>> {
    let mut extra_bytes = None;
    for (spec, data) in &batch.columns {
        if spec.dimension != LasDimension::ExtraBytes {
            continue;
        }
        let width = spec.extra_byte_width().ok_or_else(|| {
            Error::InvalidInput("ExtraBytes column requires a non-zero byte width".into())
        })?;
        if width > usize::from(u16::MAX) {
            return Err(Error::InvalidInput(format!(
                "ExtraBytes column width {width} exceeds LAS u16 range"
            )));
        }
        let values = match data {
            ColumnData::U8(values) => values.as_slice(),
            other => {
                return Err(unexpected_column_type(
                    LasDimension::ExtraBytes,
                    "U8",
                    other,
                ))
            }
        };
        if extra_bytes.replace((values, width)).is_some() {
            return Err(Error::InvalidInput(
                "ColumnBatchSource supports at most one ExtraBytes column".into(),
            ));
        }
    }
    Ok(extra_bytes)
}

fn optional_bool_column(
    batch: &LasColumnBatch,
    dimension: LasDimension,
) -> Result<Option<&[bool]>> {
    match batch.column(dimension) {
        Some(ColumnData::Bool(values)) => Ok(Some(values)),
        Some(other) => Err(unexpected_column_type(dimension, "Bool", other)),
        None => Ok(None),
    }
}

fn unexpected_column_type(dimension: LasDimension, expected: &str, actual: &ColumnData) -> Error {
    Error::InvalidInput(format!(
        "{dimension:?} column must be {expected}, found {:?}",
        actual.scalar()
    ))
}

/// Source view over a finalized writer spill file.
pub(crate) struct SpillSource<'a> {
    reader: &'a SpillReader,
    scratch: RefCell<LasPointRecord>,
}

impl<'a> SpillSource<'a> {
    pub(crate) fn new(reader: &'a SpillReader) -> Self {
        Self {
            reader,
            scratch: RefCell::new(LasPointRecord::default()),
        }
    }
}

impl CopcPointSource for SpillSource<'_> {
    fn len(&self) -> usize {
        self.reader.len()
    }

    #[inline]
    fn xyz(&self, index: usize) -> Result<(f64, f64, f64)> {
        self.reader.xyz_at(index)
    }

    fn fields_into(&self, index: usize, out: &mut CopcPointFields) -> Result<()> {
        let mut record = self.scratch.borrow_mut();
        self.reader.record_into(index, &mut record)?;
        out.x = record.x;
        out.y = record.y;
        out.z = record.z;
        out.intensity = record.intensity;
        out.return_number = record.return_number;
        out.number_of_returns = record.number_of_returns;
        out.synthetic = u8::from(record.synthetic);
        out.key_point = u8::from(record.key_point);
        out.withheld = u8::from(record.withheld);
        out.overlap = u8::from(record.overlap);
        out.scan_channel = record.scan_channel;
        out.scan_direction_flag = u8::from(record.scan_direction_flag);
        out.edge_of_flight_line = u8::from(record.edge_of_flight_line);
        out.classification = record.classification;
        out.user_data = record.user_data;
        out.scan_angle = record.scan_angle;
        out.point_source_id = record.point_source_id;
        out.gps_time = record.gps_time;
        out.red = record.red;
        out.green = record.green;
        out.blue = record.blue;
        out.extra_bytes.clear();
        out.extra_bytes.extend_from_slice(&record.extra_bytes);
        Ok(())
    }

    fn extra_byte_count(&self) -> u16 {
        self.reader.layout().extra_bytes
    }

    fn extra_bytes_vlrs(&self) -> &[las::Vlr] {
        &self.reader.layout().extra_bytes_descriptors
    }
}
