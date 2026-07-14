//! Shared COPC types used by `copc-reader` and `copc-writer`.

#![forbid(unsafe_code)]

pub mod bounds;
pub mod cancel;
pub mod columns;
pub mod error;
pub mod hierarchy;
pub mod info;
pub mod limits;
pub mod streaming;

pub use bounds::Bounds;
pub use cancel::{CancelCheck, NeverCancel};
pub use columns::{
    layout_for_las_format, ColumnData, ColumnSelection, ColumnSpec, ColumnView, LasColumnBatch,
    LasDimension, ScalarType,
};
pub use error::{Error, Result};
pub use hierarchy::{Entry, EntryAvailability, HierarchyPage, VoxelKey, HIERARCHY_ENTRY_BYTES};
pub use info::CopcInfo;
pub use limits::{MAX_EVLR_COUNT, MAX_VLR_COUNT};
pub use streaming::{
    deserialize_le, deserialize_le_into, serialize_le, LasPointRecord, StreamingLayout,
};
