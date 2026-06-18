//! Column-oriented LAS/COPC point data.

use crate::{Error, Result};

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
    ScanAngleRank,
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
            Self::WavePacketReturnPointWaveformLocation => Some(ScalarType::F32),
            Self::ScanAngleRank => Some(ScalarType::I16),
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
}

impl ColumnSpec {
    pub const fn new(dimension: LasDimension, scalar: ScalarType) -> Self {
        Self { dimension, scalar }
    }

    /// Returns the default fixed LAS/COPC scalar for `dimension`, when it has one.
    pub const fn default_for(dimension: LasDimension) -> Option<Self> {
        match dimension.default_scalar() {
            Some(scalar) => Some(Self { dimension, scalar }),
            None => None,
        }
    }

    /// Returns whether this specification has the canonical scalar for its dimension.
    pub const fn has_default_scalar(self) -> bool {
        self.dimension.accepts_scalar(self.scalar)
    }

    /// Returns whether `data` has the scalar type declared by this spec.
    pub const fn matches_data(self, data: &ColumnData) -> bool {
        self.scalar as u8 == data.scalar() as u8
    }

    /// Validate the declared scalar against the supplied data.
    pub fn validate_data(self, data: &ColumnData) -> Result<()> {
        if self.matches_data(data) {
            Ok(())
        } else {
            Err(Error::InvalidInput(format!(
                "column {:?} declares {:?} data but contains {:?}",
                self.dimension,
                self.scalar,
                data.scalar()
            )))
        }
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

/// A column-oriented batch of LAS/COPC point values.
#[derive(Clone, Debug, PartialEq)]
pub struct LasColumnBatch {
    pub len: usize,
    pub columns: Vec<(ColumnSpec, ColumnData)>,
}

impl LasColumnBatch {
    pub fn new(columns: Vec<(ColumnSpec, ColumnData)>) -> Result<Self> {
        let len = columns.first().map(|(_, data)| data.len()).unwrap_or(0);
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
        for (spec, data) in &self.columns {
            spec.validate_data(data)?;
            if data.len() != self.len {
                return Err(Error::InvalidInput(format!(
                    "column {:?} has {} values but batch len is {}",
                    spec.dimension,
                    data.len(),
                    self.len
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
    fn default_scalar_validation_allows_extra_bytes() {
        assert_eq!(
            ColumnSpec::new(LasDimension::GpsTime, ScalarType::F64),
            ColumnSpec::default_for(LasDimension::GpsTime).unwrap()
        );
        assert!(ColumnSpec::new(LasDimension::ExtraBytes, ScalarType::I32).has_default_scalar());
        assert!(
            ColumnSpec::new(LasDimension::ScanAngleRank, ScalarType::F32)
                .validate_default_scalar()
                .is_err()
        );
    }
}
