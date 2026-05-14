use thiserror::Error as ThisError;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, ThisError)]
pub enum Error {
    #[error("invalid COPC input: {0}")]
    InvalidInput(String),
    #[error("invalid COPC data: {0}")]
    InvalidData(String),
    #[error("LAS/LAZ error: {0}")]
    Las(String),
    #[error("{context}: {source}")]
    Io {
        context: &'static str,
        #[source]
        source: std::io::Error,
    },
    #[error("operation cancelled")]
    Cancelled,
    #[error("unsupported COPC feature: {0}")]
    Unsupported(String),
}

impl Error {
    pub fn io(context: &'static str, source: std::io::Error) -> Self {
        Self::Io { context, source }
    }
}
