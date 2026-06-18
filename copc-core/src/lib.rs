//! Shared COPC types used by `copc-reader` and `copc-writer`.

#![forbid(unsafe_code)]

pub mod bounds;
pub mod cancel;
pub mod columns;
pub mod error;
pub mod hierarchy;
pub mod info;
pub mod streaming;

pub use bounds::Bounds;
pub use cancel::{CancelCheck, NeverCancel};
pub use columns::{
    layout_for_las_format, scan_angle_rank_from_degrees, ColumnData, ColumnSelection, ColumnSpec,
    ColumnView, LasColumnBatch, LasDimension, ScalarType,
};
pub use error::{Error, Result};
pub use hierarchy::{Entry, EntryAvailability, HierarchyPage, VoxelKey, HIERARCHY_ENTRY_BYTES};
pub use info::CopcInfo;
pub use streaming::{deserialize_le, serialize_le, LasPointRecord, StreamingLayout};
