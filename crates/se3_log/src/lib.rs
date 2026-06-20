//! Shared logforth setup for SE3 runtime binaries.

use std::{
    path::{Path, PathBuf},
    sync::{
        Arc, OnceLock,
        atomic::{AtomicBool, Ordering},
    },
};

use jiff::Zoned;
use log::info;
use logforth::append;
use logforth::bridge::log::LogBridge;
use logforth::core::Logger;
use logforth::filter::RustLogFilter;
use logforth::layout::TextLayout;
use logforth_append_async::AsyncBuilder;
use logforth_append_file::FileBuilder;
use serde::{Deserialize, Serialize};
use thiserror::Error;

static LOG_INITIALIZED: AtomicBool = AtomicBool::new(false);
static LOGGER: OnceLock<Arc<Logger>> = OnceLock::new();
const DEPLOY_LOG_ROOT: &str = "/var/opt/se3/logs";
const LOG_DIR_ENV: &str = "SE3_LOG_DIR";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct LoggerConfig {
    pub console_log_filter: String,
    pub file_log_filter: String,
    pub console_log_enable: bool,
    pub file_log_enable: bool,
}

impl LoggerConfig {
    pub fn new(
        console_log_filter: impl Into<String>,
        file_log_filter: impl Into<String>,
        console_log_enable: bool,
        file_log_enable: bool,
    ) -> Self {
        Self {
            console_log_filter: console_log_filter.into(),
            file_log_filter: file_log_filter.into(),
            console_log_enable,
            file_log_enable,
        }
    }
}

impl Default for LoggerConfig {
    fn default() -> Self {
        Self::new("info", "info", true, true)
    }
}

#[derive(Debug, Error)]
pub enum LogError {
    #[error("failed to create log directory `{}`", .path.display())]
    CreateLogDirectory {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to build file logger: {0}")]
    FileLogger(logforth::Error),
    #[error("failed to initialize global logger: {0}")]
    SetLogger(#[from] log::SetLoggerError),
}

pub type LogResult<T> = Result<T, LogError>;

pub struct LoggerGuard;

impl Drop for LoggerGuard {
    fn drop(&mut self) {
        flush();
    }
}

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")))
}

fn local_log_root() -> PathBuf {
    workspace_root().join("log")
}

fn log_root() -> PathBuf {
    if let Some(path) = std::env::var_os(LOG_DIR_ENV) {
        return PathBuf::from(path);
    }

    let deploy_root = PathBuf::from(DEPLOY_LOG_ROOT);
    if deploy_root.exists() {
        deploy_root
    } else {
        local_log_root()
    }
}

fn daily_log_dir(now: &Zoned) -> PathBuf {
    log_root()
        .join(format!("{:04}", now.year()))
        .join(format!("{:02}", now.month()))
        .join(format!("{:02}", now.day()))
}

pub fn init(config: &LoggerConfig) -> LogResult<Option<LoggerGuard>> {
    if !config.file_log_enable && !config.console_log_enable {
        return Ok(None);
    }

    let mut logger_builder = logforth::core::builder();

    if config.console_log_enable {
        let console_filter: RustLogFilter = config.console_log_filter.as_str().into();
        let console_appender = append::Stdout::default().with_layout(TextLayout::default());
        logger_builder = logger_builder
            .dispatch(|dispatch| dispatch.filter(console_filter).append(console_appender));
    }

    if config.file_log_enable {
        let file_filter: RustLogFilter = config.file_log_filter.as_str().into();
        let now = Zoned::now();
        let file_name = format!("{:02}:{:02}:{:02}", now.hour(), now.minute(), now.second());
        let directory_name = daily_log_dir(&now);
        std::fs::create_dir_all(&directory_name).map_err(|source| {
            LogError::CreateLogDirectory {
                path: directory_name.clone(),
                source,
            }
        })?;

        let file_appender = FileBuilder::new(directory_name, file_name)
            .layout(TextLayout::default().no_color())
            .build()
            .map_err(LogError::FileLogger)?;

        let async_file_appender = AsyncBuilder::new("se3-log-file")
            .buffered_lines_limit(Some(8192))
            .overflow_block()
            .append(file_appender)
            .build();

        logger_builder = logger_builder
            .dispatch(|dispatch| dispatch.filter(file_filter).append(async_file_appender));
    }

    let logger = Arc::new(logger_builder.build());
    let bridge = LogBridge::new(logger.clone());
    log::set_boxed_logger(Box::new(bridge))?;
    log::set_max_level(log::LevelFilter::Trace);
    let _ = LOGGER.set(logger.clone());
    LOG_INITIALIZED.store(true, Ordering::SeqCst);

    info!(
        "log initialized with console filter: {}",
        config.console_log_filter
    );
    info!(
        "log initialized with file filter: {}",
        config.file_log_filter
    );
    info!(
        "log initialized with file_log_enable: {}, console_log_enable: {}",
        config.file_log_enable, config.console_log_enable
    );

    Ok(Some(LoggerGuard))
}

pub fn flush() {
    if let Some(logger) = LOGGER.get() {
        logger.flush();
    }
}

pub fn is_initialized() -> bool {
    LOG_INITIALIZED.load(Ordering::SeqCst)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_enables_console_and_file_logging() {
        let config = LoggerConfig::default();

        assert_eq!(config.console_log_filter, "info");
        assert_eq!(config.file_log_filter, "info");
        assert!(config.console_log_enable);
        assert!(config.file_log_enable);
    }

    #[test]
    fn local_log_root_uses_workspace_log_dir() {
        assert_eq!(local_log_root(), workspace_root().join("log"));
    }
}
