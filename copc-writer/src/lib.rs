//! Pure-Rust COPC writer.

mod hierarchy_pages;
mod las_out;
mod lod;
mod metadata;
mod source;
mod spill;
mod validate;
mod writer;

pub(crate) const CANCEL_POLL_STRIDE: usize = 4_096;

pub use metadata::CopcWriteMetadata;
pub use source::{ColumnBatchSource, CopcPointFields, CopcPointSource};
pub use spill::{SpillReader, SpillWriter};
pub use writer::{
    convert_las_to_copc_streaming, convert_las_to_copc_streaming_with_crs_wkt_override,
    write_source, write_source_with_cancel, write_streaming_with_cancel, CopcWriterParams,
};
