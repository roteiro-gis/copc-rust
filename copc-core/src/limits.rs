//! Shared parsing limits applied to untrusted LAS/COPC input.

/// Maximum number of VLRs accepted when parsing LAS/COPC input.
pub const MAX_VLR_COUNT: u32 = 4_096;

/// Maximum number of EVLRs accepted when parsing LAS/COPC input.
pub const MAX_EVLR_COUNT: u32 = 4_096;
