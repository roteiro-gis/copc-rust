//! Column-oriented LAS/COPC point data.

use std::collections::HashSet;

use crate::{Error, Result};

use las::point::Format as LasPointFormat;

/// LAS/COPC point dimensions that can be represented as columns.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LasDimension {
    X,
    Y,
    Z,
    Intensity,
    ReturnNumber,
    NumberOfReturns,
    Classification,
    ScanDirectionFlag,
    EdgeOfFlightLine,
    /// Scan angle in degrees. LAS 1.4 stores it as a scaled i16 in 0.006°
    /// increments; the column carries the decoded degrees losslessly.
    ScanAngle,
    UserData,
    PointSourceId,
    Synthetic,
    KeyPoint,
    Withheld,
    Overlap,
    ScanChannel,
    GpsTime,
    Red,
    Green,
    Blue,
    Nir,
    WaveformPacketDescriptorIndex,
    WaveformPacketByteOffset,
    WaveformPacketSize,
    WavePacketReturnPointWaveformLocation,
    ExtraBytes,
}

impl LasDimension {
    /// The default scalar representation for fixed LAS/COPC dimensions.
    pub const fn default_scalar(self) -> Option<ScalarType> {
        match self {
            Self::X | Self::Y | Self::Z | Self::GpsTime => Some(ScalarType::F64),
            Self::ScanAngle | Self::WavePacketReturnPointWaveformLocation => Some(ScalarType::F32),
            Self::WaveformPacketByteOffset => Some(ScalarType::U64),
            Self::Intensity
            | Self::PointSourceId
            | Self::Red
            | Self::Green
            | Self::Blue
            | Self::Nir => Some(ScalarType::U16),
            Self::WaveformPacketSize => Some(ScalarType::U32),
            Self::ReturnNumber
            | Self::NumberOfReturns
            | Self::Classification
            | Self::UserData
            | Self::ScanChannel
            | Self::WaveformPacketDescriptorIndex => Some(ScalarType::U8),
            Self::ScanDirectionFlag
            | Self::EdgeOfFlightLine
            | Self::Synthetic
            | Self::KeyPoint
            | Self::Withheld
            | Self::Overlap => Some(ScalarType::Bool),
            Self::ExtraBytes => None,
        }
    }

    /// Returns whether `scalar` is the default fixed-width representation for this dimension.
    pub const fn accepts_scalar(self, scalar: ScalarType) -> bool {
        match self.default_scalar() {
            Some(default) => default as u8 == scalar as u8,
            None => true,
        }
    }
}

/// Requested LAS/COPC dimensions for column-oriented reads.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ColumnSelection {
    dimensions: Vec<LasDimension>,
}

impl ColumnSelection {
    pub fn all() -> Self {
        Self::from_dimensions([
            LasDimension::X,
            LasDimension::Y,
            LasDimension::Z,
            LasDimension::Intensity,
            LasDimension::ReturnNumber,
            LasDimension::NumberOfReturns,
            LasDimension::Classification,
            LasDimension::ScanDirectionFlag,
            LasDimension::EdgeOfFlightLine,
            LasDimension::ScanAngle,
            LasDimension::UserData,
            LasDimension::PointSourceId,
            LasDimension::Synthetic,
            LasDimension::KeyPoint,
            LasDimension::Withheld,
            LasDimension::Overlap,
            LasDimension::ScanChannel,
            LasDimension::GpsTime,
            LasDimension::Red,
            LasDimension::Green,
            LasDimension::Blue,
            LasDimension::Nir,
            LasDimension::WaveformPacketDescriptorIndex,
            LasDimension::WaveformPacketByteOffset,
            LasDimension::WaveformPacketSize,
            LasDimension::WavePacketReturnPointWaveformLocation,
            LasDimension::ExtraBytes,
        ])
    }

    pub fn xyz() -> Self {
        Self::from_dimensions([LasDimension::X, LasDimension::Y, LasDimension::Z])
    }

    pub fn from_dimensions<I>(dims: I) -> Self
    where
        I: IntoIterator<Item = LasDimension>,
    {
        let mut dimensions = Vec::new();
        for dim in dims {
            if !dimensions.contains(&dim) {
                dimensions.push(dim);
            }
        }
        Self { dimensions }
    }

    pub fn contains(&self, dim: LasDimension) -> bool {
        self.dimensions.contains(&dim)
    }

    pub fn dimensions(&self) -> &[LasDimension] {
        &self.dimensions
    }

    pub fn len(&self) -> usize {
        self.dimensions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.dimensions.is_empty()
    }
}

/// Primitive scalar types supported by LAS/COPC column data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ScalarType {
    F64,
    F32,
    I64,
    I32,
    I16,
    I8,
    U64,
    U32,
    U16,
    U8,
    Bool,
}

/// Declares the LAS/COPC dimension and scalar type for a column.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ColumnSpec {
    pub dimension: LasDimension,
    pub scalar: ScalarType,
    /// For `LasDimension::ExtraBytes`, the fixed byte count stored for each point.
    pub byte_width: Option<usize>,
}

impl ColumnSpec {
    pub const fn new(dimension: LasDimension, scalar: ScalarType) -> Self {
        Self {
            dimension,
            scalar,
            byte_width: None,
        }
    }

    pub const fn extra_bytes(byte_width: usize) -> Self {
        Self {
            dimension: LasDimension::ExtraBytes,
            scalar: ScalarType::U8,
            byte_width: Some(byte_width),
        }
    }

    /// Returns the default fixed LAS/COPC scalar for `dimension`, when it has one.
    pub const fn default_for(dimension: LasDimension) -> Option<Self> {
        match dimension.default_scalar() {
            Some(scalar) => Some(Self {
                dimension,
                scalar,
                byte_width: None,
            }),
            None => None,
        }
    }

    /// Returns whether this specification has the canonical scalar for its dimension.
    pub const fn has_default_scalar(self) -> bool {
        if matches!(self.dimension, LasDimension::ExtraBytes) {
            matches!(self.scalar, ScalarType::U8) && self.byte_width.is_some()
        } else {
            self.byte_width.is_none() && self.dimension.accepts_scalar(self.scalar)
        }
    }

    /// Returns whether `data` has the scalar type declared by this spec.
    pub const fn matches_data(self, data: &ColumnData) -> bool {
        self.scalar as u8 == data.scalar() as u8
    }

    /// Validate the declared scalar against the supplied data.
    pub fn validate_data(self, data: &ColumnData) -> Result<()> {
        if !self.matches_data(data) {
            return Err(Error::InvalidInput(format!(
                "column {:?} declares {:?} data but contains {:?}",
                self.dimension,
                self.scalar,
                data.scalar()
            )));
        }
        if self.dimension == LasDimension::ExtraBytes && self.extra_byte_width().is_none() {
            return Err(Error::InvalidInput(
                "ExtraBytes column requires a non-zero byte width".into(),
            ));
        }
        if self.dimension != LasDimension::ExtraBytes && self.byte_width.is_some() {
            return Err(Error::InvalidInput(format!(
                "column {:?} cannot declare byte width {:?}",
                self.dimension, self.byte_width
            )));
        }
        Ok(())
    }

    /// Validate that this spec uses the fixed LAS/COPC scalar for its dimension.
    pub fn validate_default_scalar(self) -> Result<()> {
        if self.has_default_scalar() {
            Ok(())
        } else {
            Err(Error::InvalidInput(format!(
                "column {:?} declares {:?}, expected {:?}",
                self.dimension,
                self.scalar,
                self.dimension.default_scalar()
            )))
        }
    }

    pub fn extra_byte_width(self) -> Option<usize> {
        match (self.dimension, self.scalar, self.byte_width) {
            (LasDimension::ExtraBytes, ScalarType::U8, Some(width)) if width > 0 => Some(width),
            _ => None,
        }
    }

    pub fn point_count_for_data(self, data: &ColumnData) -> Result<usize> {
        self.validate_data(data)?;
        if self.dimension == LasDimension::ExtraBytes {
            let width = self.extra_byte_width().ok_or_else(|| {
                Error::InvalidInput("ExtraBytes column requires a non-zero byte width".into())
            })?;
            if data.len() % width != 0 {
                return Err(Error::InvalidInput(format!(
                    "ExtraBytes column has {} bytes, which is not divisible by byte width {width}",
                    data.len()
                )));
            }
            Ok(data.len() / width)
        } else {
            Ok(data.len())
        }
    }
}

/// Owned column values.
#[derive(Clone, Debug, PartialEq)]
pub enum ColumnData {
    F64(Vec<f64>),
    F32(Vec<f32>),
    I64(Vec<i64>),
    I32(Vec<i32>),
    I16(Vec<i16>),
    I8(Vec<i8>),
    U64(Vec<u64>),
    U32(Vec<u32>),
    U16(Vec<u16>),
    U8(Vec<u8>),
    Bool(Vec<bool>),
}

impl ColumnData {
    pub fn len(&self) -> usize {
        match self {
            Self::F64(values) => values.len(),
            Self::F32(values) => values.len(),
            Self::I64(values) => values.len(),
            Self::I32(values) => values.len(),
            Self::I16(values) => values.len(),
            Self::I8(values) => values.len(),
            Self::U64(values) => values.len(),
            Self::U32(values) => values.len(),
            Self::U16(values) => values.len(),
            Self::U8(values) => values.len(),
            Self::Bool(values) => values.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub const fn scalar(&self) -> ScalarType {
        match self {
            Self::F64(_) => ScalarType::F64,
            Self::F32(_) => ScalarType::F32,
            Self::I64(_) => ScalarType::I64,
            Self::I32(_) => ScalarType::I32,
            Self::I16(_) => ScalarType::I16,
            Self::I8(_) => ScalarType::I8,
            Self::U64(_) => ScalarType::U64,
            Self::U32(_) => ScalarType::U32,
            Self::U16(_) => ScalarType::U16,
            Self::U8(_) => ScalarType::U8,
            Self::Bool(_) => ScalarType::Bool,
        }
    }

    pub const fn scalar_type(&self) -> ScalarType {
        self.scalar()
    }

    pub const fn matches_scalar(&self, scalar: ScalarType) -> bool {
        self.scalar() as u8 == scalar as u8
    }

    pub fn view(&self) -> ColumnView<'_> {
        match self {
            Self::F64(values) => ColumnView::F64(values),
            Self::F32(values) => ColumnView::F32(values),
            Self::I64(values) => ColumnView::I64(values),
            Self::I32(values) => ColumnView::I32(values),
            Self::I16(values) => ColumnView::I16(values),
            Self::I8(values) => ColumnView::I8(values),
            Self::U64(values) => ColumnView::U64(values),
            Self::U32(values) => ColumnView::U32(values),
            Self::U16(values) => ColumnView::U16(values),
            Self::U8(values) => ColumnView::U8(values),
            Self::Bool(values) => ColumnView::Bool(values),
        }
    }
}

/// Borrowed column values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ColumnView<'a> {
    F64(&'a [f64]),
    F32(&'a [f32]),
    I64(&'a [i64]),
    I32(&'a [i32]),
    I16(&'a [i16]),
    I8(&'a [i8]),
    U64(&'a [u64]),
    U32(&'a [u32]),
    U16(&'a [u16]),
    U8(&'a [u8]),
    Bool(&'a [bool]),
}

impl ColumnView<'_> {
    pub fn len(&self) -> usize {
        match self {
            Self::F64(values) => values.len(),
            Self::F32(values) => values.len(),
            Self::I64(values) => values.len(),
            Self::I32(values) => values.len(),
            Self::I16(values) => values.len(),
            Self::I8(values) => values.len(),
            Self::U64(values) => values.len(),
            Self::U32(values) => values.len(),
            Self::U16(values) => values.len(),
            Self::U8(values) => values.len(),
            Self::Bool(values) => values.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub const fn scalar(&self) -> ScalarType {
        match self {
            Self::F64(_) => ScalarType::F64,
            Self::F32(_) => ScalarType::F32,
            Self::I64(_) => ScalarType::I64,
            Self::I32(_) => ScalarType::I32,
            Self::I16(_) => ScalarType::I16,
            Self::I8(_) => ScalarType::I8,
            Self::U64(_) => ScalarType::U64,
            Self::U32(_) => ScalarType::U32,
            Self::U16(_) => ScalarType::U16,
            Self::U8(_) => ScalarType::U8,
            Self::Bool(_) => ScalarType::Bool,
        }
    }

    pub const fn scalar_type(&self) -> ScalarType {
        self.scalar()
    }
}

/// Returns the column layout available from a LAS point format.
pub fn layout_for_las_format(format: LasPointFormat) -> Vec<ColumnSpec> {
    let mut columns = Vec::with_capacity(27);

    push_default_specs(
        &mut columns,
        [
            LasDimension::X,
            LasDimension::Y,
            LasDimension::Z,
            LasDimension::Intensity,
            LasDimension::ReturnNumber,
            LasDimension::NumberOfReturns,
            LasDimension::Classification,
            LasDimension::ScanDirectionFlag,
            LasDimension::EdgeOfFlightLine,
            LasDimension::ScanAngle,
            LasDimension::UserData,
            LasDimension::PointSourceId,
            LasDimension::Synthetic,
            LasDimension::KeyPoint,
            LasDimension::Withheld,
            LasDimension::Overlap,
            LasDimension::ScanChannel,
        ],
    );

    if format.has_gps_time {
        columns.push(default_column_spec(LasDimension::GpsTime));
    }
    if format.has_color {
        push_default_specs(
            &mut columns,
            [LasDimension::Red, LasDimension::Green, LasDimension::Blue],
        );
    }
    if format.has_nir {
        columns.push(default_column_spec(LasDimension::Nir));
    }
    if format.has_waveform {
        push_default_specs(
            &mut columns,
            [
                LasDimension::WaveformPacketDescriptorIndex,
                LasDimension::WaveformPacketByteOffset,
                LasDimension::WaveformPacketSize,
                LasDimension::WavePacketReturnPointWaveformLocation,
            ],
        );
    }
    if format.extra_bytes > 0 {
        columns.push(ColumnSpec::extra_bytes(usize::from(format.extra_bytes)));
    }
    columns
}

fn push_default_specs<I>(columns: &mut Vec<ColumnSpec>, dims: I)
where
    I: IntoIterator<Item = LasDimension>,
{
    columns.extend(dims.into_iter().map(default_column_spec));
}

fn default_column_spec(dimension: LasDimension) -> ColumnSpec {
    ColumnSpec::default_for(dimension).expect("fixed LAS dimension has a default scalar")
}

/// A column-oriented batch of LAS/COPC point values.
#[derive(Clone, Debug, PartialEq)]
pub struct LasColumnBatch {
    pub len: usize,
    pub columns: Vec<(ColumnSpec, ColumnData)>,
}

impl LasColumnBatch {
    pub fn new(columns: Vec<(ColumnSpec, ColumnData)>) -> Result<Self> {
        let len = match columns.first() {
            Some((spec, data)) => spec.point_count_for_data(data)?,
            None => 0,
        };
        let batch = Self { len, columns };
        batch.validate()?;
        Ok(batch)
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn column(&self, dimension: LasDimension) -> Option<&ColumnData> {
        self.columns
            .iter()
            .find_map(|(spec, data)| (spec.dimension == dimension).then_some(data))
    }

    pub fn column_by_spec(&self, spec: ColumnSpec) -> Option<&ColumnData> {
        self.columns
            .iter()
            .find_map(|(column_spec, data)| (*column_spec == spec).then_some(data))
    }

    pub fn column_view(&self, dimension: LasDimension) -> Option<ColumnView<'_>> {
        self.column(dimension).map(ColumnData::view)
    }

    pub fn column_view_by_spec(&self, spec: ColumnSpec) -> Option<ColumnView<'_>> {
        self.column_by_spec(spec).map(ColumnData::view)
    }

    /// Validate scalar declarations and column lengths for this batch.
    pub fn validate(&self) -> Result<()> {
        let mut dimensions = HashSet::with_capacity(self.columns.len());
        for (spec, data) in &self.columns {
            if !dimensions.insert(spec.dimension) {
                return Err(Error::InvalidInput(format!(
                    "column {:?} appears more than once in the batch",
                    spec.dimension
                )));
            }
            let point_count = spec.point_count_for_data(data)?;
            if point_count != self.len {
                return Err(Error::InvalidInput(format!(
                    "column {:?} has {} points but batch len is {}",
                    spec.dimension, point_count, self.len
                )));
            }
        }
        Ok(())
    }

    /// Validate scalar declarations, fixed LAS/COPC scalar choices, and column lengths.
    pub fn validate_default_scalars(&self) -> Result<()> {
        self.validate()?;
        for (spec, _) in &self.columns {
            spec.validate_default_scalar()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_layout_dims() -> Vec<LasDimension> {
        vec![
            LasDimension::X,
            LasDimension::Y,
            LasDimension::Z,
            LasDimension::Intensity,
            LasDimension::ReturnNumber,
            LasDimension::NumberOfReturns,
            LasDimension::Classification,
            LasDimension::ScanDirectionFlag,
            LasDimension::EdgeOfFlightLine,
            LasDimension::ScanAngle,
            LasDimension::UserData,
            LasDimension::PointSourceId,
            LasDimension::Synthetic,
            LasDimension::KeyPoint,
            LasDimension::Withheld,
            LasDimension::Overlap,
            LasDimension::ScanChannel,
        ]
    }

    fn assert_layout_dims(format_id: u8, expected: Vec<LasDimension>) {
        let format = LasPointFormat::new(format_id).unwrap();
        let layout = layout_for_las_format(format);
        let dims: Vec<_> = layout.iter().map(|spec| spec.dimension).collect();
        assert_eq!(expected, dims, "format {format_id}");
        for spec in layout {
            spec.validate_default_scalar().unwrap();
        }
    }

    #[test]
    fn data_reports_len_and_scalar() {
        let data = ColumnData::U16(vec![10, 20, 30]);

        assert_eq!(3, data.len());
        assert!(!data.is_empty());
        assert_eq!(ScalarType::U16, data.scalar());
        assert!(data.matches_scalar(ScalarType::U16));
    }

    #[test]
    fn batch_finds_owned_columns_and_views() {
        let batch = LasColumnBatch::new(vec![
            (
                ColumnSpec::new(LasDimension::X, ScalarType::F64),
                ColumnData::F64(vec![1.0, 2.0]),
            ),
            (
                ColumnSpec::new(LasDimension::Intensity, ScalarType::U16),
                ColumnData::U16(vec![100, 200]),
            ),
            (
                ColumnSpec::new(LasDimension::Withheld, ScalarType::Bool),
                ColumnData::Bool(vec![false, true]),
            ),
        ])
        .unwrap();

        assert_eq!(2, batch.len());
        assert!(!batch.is_empty());
        assert_eq!(
            Some(&ColumnData::U16(vec![100, 200])),
            batch.column(LasDimension::Intensity)
        );
        assert_eq!(
            Some(ColumnView::Bool(&[false, true])),
            batch.column_view(LasDimension::Withheld)
        );
    }

    #[test]
    fn batch_rejects_scalar_mismatch() {
        let err = LasColumnBatch::new(vec![(
            ColumnSpec::new(LasDimension::Intensity, ScalarType::U16),
            ColumnData::U8(vec![1, 2]),
        )])
        .unwrap_err();

        assert!(err
            .to_string()
            .contains("declares U16 data but contains U8"));
    }

    #[test]
    fn batch_rejects_len_mismatch() {
        let batch = LasColumnBatch {
            len: 3,
            columns: vec![(
                ColumnSpec::new(LasDimension::X, ScalarType::F64),
                ColumnData::F64(vec![1.0, 2.0]),
            )],
        };

        assert!(batch.validate().is_err());
    }

    #[test]
    fn batch_rejects_duplicate_dimensions() {
        let batch = LasColumnBatch::new(vec![
            (
                ColumnSpec::new(LasDimension::Intensity, ScalarType::U16),
                ColumnData::U16(vec![1]),
            ),
            (
                ColumnSpec::new(LasDimension::Intensity, ScalarType::U16),
                ColumnData::U16(vec![2]),
            ),
        ]);

        assert!(batch.unwrap_err().to_string().contains("more than once"));
    }

    #[test]
    fn batch_validates_fixed_width_extra_bytes() {
        let batch = LasColumnBatch::new(vec![(
            ColumnSpec::extra_bytes(3),
            ColumnData::U8(vec![1, 2, 3, 4, 5, 6]),
        )])
        .unwrap();

        assert_eq!(2, batch.len());
        assert_eq!(
            Some(&ColumnData::U8(vec![1, 2, 3, 4, 5, 6])),
            batch.column(LasDimension::ExtraBytes)
        );

        let invalid = LasColumnBatch::new(vec![(
            ColumnSpec::extra_bytes(3),
            ColumnData::U8(vec![1, 2, 3, 4]),
        )]);
        assert!(invalid.is_err());

        let missing_width = LasColumnBatch::new(vec![(
            ColumnSpec::new(LasDimension::ExtraBytes, ScalarType::U8),
            ColumnData::U8(vec![1, 2, 3]),
        )]);
        assert!(missing_width.is_err());
    }

    #[test]
    fn default_scalar_validation_allows_extra_bytes() {
        assert_eq!(
            ColumnSpec::new(LasDimension::GpsTime, ScalarType::F64),
            ColumnSpec::default_for(LasDimension::GpsTime).unwrap()
        );
        assert!(ColumnSpec::extra_bytes(4).has_default_scalar());
        assert!(ColumnSpec::new(LasDimension::ExtraBytes, ScalarType::U8)
            .validate_default_scalar()
            .is_err());
        assert!(ColumnSpec::new(LasDimension::ScanAngle, ScalarType::I16)
            .validate_default_scalar()
            .is_err());
        assert!(ColumnSpec::new(LasDimension::ScanAngle, ScalarType::F32)
            .validate_default_scalar()
            .is_ok());
    }

    #[test]
    fn selection_tracks_requested_dimensions() {
        let xyz = ColumnSelection::xyz();
        assert_eq!(
            &[LasDimension::X, LasDimension::Y, LasDimension::Z],
            xyz.dimensions()
        );
        assert!(xyz.contains(LasDimension::X));
        assert!(!xyz.contains(LasDimension::Intensity));

        let selection = ColumnSelection::from_dimensions([
            LasDimension::Intensity,
            LasDimension::X,
            LasDimension::Intensity,
        ]);
        assert_eq!(
            &[LasDimension::Intensity, LasDimension::X],
            selection.dimensions()
        );
        assert_eq!(2, selection.len());
        assert!(!selection.is_empty());

        let all = ColumnSelection::all();
        assert!(all.contains(LasDimension::WaveformPacketByteOffset));
        assert!(all.contains(LasDimension::ExtraBytes));
    }

    #[test]
    fn layout_for_format_0_has_core_dimensions() {
        assert_layout_dims(0, base_layout_dims());
    }

    #[test]
    fn layout_for_format_3_adds_gps_and_color() {
        let mut expected = base_layout_dims();
        expected.extend([
            LasDimension::GpsTime,
            LasDimension::Red,
            LasDimension::Green,
            LasDimension::Blue,
        ]);

        assert_layout_dims(3, expected);
    }

    #[test]
    fn layout_for_format_6_adds_gps() {
        let mut expected = base_layout_dims();
        expected.push(LasDimension::GpsTime);

        assert_layout_dims(6, expected);
    }

    #[test]
    fn layout_for_format_7_adds_gps_and_color() {
        let mut expected = base_layout_dims();
        expected.extend([
            LasDimension::GpsTime,
            LasDimension::Red,
            LasDimension::Green,
            LasDimension::Blue,
        ]);

        assert_layout_dims(7, expected);
    }

    #[test]
    fn layout_for_format_8_adds_gps_color_and_nir() {
        let mut expected = base_layout_dims();
        expected.extend([
            LasDimension::GpsTime,
            LasDimension::Red,
            LasDimension::Green,
            LasDimension::Blue,
            LasDimension::Nir,
        ]);

        assert_layout_dims(8, expected);
    }

    #[test]
    fn layout_for_format_10_adds_all_optional_las_dimensions() {
        let mut expected = base_layout_dims();
        expected.extend([
            LasDimension::GpsTime,
            LasDimension::Red,
            LasDimension::Green,
            LasDimension::Blue,
            LasDimension::Nir,
            LasDimension::WaveformPacketDescriptorIndex,
            LasDimension::WaveformPacketByteOffset,
            LasDimension::WaveformPacketSize,
            LasDimension::WavePacketReturnPointWaveformLocation,
        ]);

        assert_layout_dims(10, expected);
    }

    #[test]
    fn layout_includes_extra_bytes_with_byte_width_when_format_declares_them() {
        let mut format = LasPointFormat::new(0).unwrap();
        format.extra_bytes = 4;

        let layout = layout_for_las_format(format);

        assert_eq!(Some(&ColumnSpec::extra_bytes(4)), layout.last());
    }
}
