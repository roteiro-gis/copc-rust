use crate::Result;

/// Cancellation hook used by streaming readers and writers.
pub trait CancelCheck {
    fn check(&self) -> Result<()>;
}

/// Cancellation hook that never cancels.
#[derive(Debug, Clone, Copy, Default)]
pub struct NeverCancel;

impl CancelCheck for NeverCancel {
    fn check(&self) -> Result<()> {
        Ok(())
    }
}
