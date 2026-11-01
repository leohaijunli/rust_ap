//! Host-testable logger control and configuration contract.

pub const MIN_SYNC_PERIOD_MS: u32 = 100;
pub const MAX_SYNC_PERIOD_MS: u32 = 10_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub sync_period_ms: u32,
}

impl Config {
    pub const DEFAULT: Self = Self {
        sync_period_ms: 1_000,
    };

    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.sync_period_ms < MIN_SYNC_PERIOD_MS || self.sync_period_ms > MAX_SYNC_PERIOD_MS {
            return Err(ConfigError::SyncPeriod);
        }
        Ok(self)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    SyncPeriod,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Disabled,
    Initializing,
    Ready,
    Stopping,
    Error,
    Managing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Start,
    Stop,
    Sync,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleCommand {
    Status,
    Start,
    Stop,
    Sync,
    Config(Config),
}

pub fn parse_console(line: &[u8]) -> Option<ConsoleCommand> {
    match line {
        b"!logger" | b"logger" | b"logger status" => Some(ConsoleCommand::Status),
        b"!logger-start" | b"logger start" => Some(ConsoleCommand::Start),
        b"!logger-stop" | b"logger stop" => Some(ConsoleCommand::Stop),
        b"!logger-sync" | b"logger sync" => Some(ConsoleCommand::Sync),
        _ => line
            .strip_prefix(b"!logger-config ")
            .or_else(|| line.strip_prefix(b"logger config "))
            .and_then(|value| core::str::from_utf8(value).ok())
            .and_then(|value| value.parse().ok())
            .map(|sync_period_ms| ConsoleCommand::Config(Config { sync_period_ms })),
    }
}

pub const fn accepts(status: Status, command: Command) -> bool {
    match command {
        Command::Start => matches!(status, Status::Disabled | Status::Error),
        Command::Stop | Command::Sync => matches!(status, Status::Ready),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlError {
    InvalidState,
    QueueFull,
    InvalidConfig(ConfigError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Link {
    Usb = 1,
    Uart = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogRequest {
    List { start: u16, end: u16 },
    Data { id: u16, offset: u32, count: u32 },
    End,
    Erase,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RoutedLogRequest {
    pub link: Link,
    pub request: LogRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogError {
    Busy,
    Mount,
    Directory,
    NotFound,
    Range,
    Read,
    EraseDisabled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogResponse {
    Entry {
        id: u16,
        num_logs: u16,
        last_log_num: u16,
        size: u32,
        time_utc: u32,
    },
    Data {
        id: u16,
        offset: u32,
        count: u8,
        data: [u8; 90],
    },
    Error(LogError),
    Done,
}

pub fn is_ulog_name(name: &[u8]) -> bool {
    name.len() >= 4 && name[name.len() - 4..].eq_ignore_ascii_case(b".ULG")
}

pub const fn list_bounds(num_logs: u16, start: u16, end: u16) -> Option<(u16, u16)> {
    if num_logs == 0 || start >= num_logs {
        return None;
    }
    let last_log_num = num_logs - 1;
    let last = if end == u16::MAX || end > last_log_num {
        last_log_num
    } else {
        end
    };
    if start > last {
        None
    } else {
        Some((start, last))
    }
}

pub const fn download_count(size: u32, offset: u32, requested: u32) -> Result<u32, LogError> {
    if offset >= size {
        return Err(LogError::Range);
    }
    let available = size - offset;
    Ok(if requested == u32::MAX || requested > available {
        available
    } else {
        requested
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub status: Status,
    pub stage: u8,
    pub file_index: u32,
    pub bytes_written: u32,
    pub flushes: u32,
    pub errors: u32,
    pub config: Config,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_period_has_bounded_public_contract() {
        assert_eq!(Config::DEFAULT.validate(), Ok(Config::DEFAULT));
        for sync_period_ms in [0, MIN_SYNC_PERIOD_MS - 1, MAX_SYNC_PERIOD_MS + 1, u32::MAX] {
            assert_eq!(
                Config { sync_period_ms }.validate(),
                Err(ConfigError::SyncPeriod)
            );
        }
        for sync_period_ms in [MIN_SYNC_PERIOD_MS, 250, 1_000, MAX_SYNC_PERIOD_MS] {
            let config = Config { sync_period_ms };
            assert_eq!(config.validate(), Ok(config));
        }
    }

    #[test]
    fn console_commands_and_state_transitions_are_explicit() {
        assert_eq!(parse_console(b"logger start"), Some(ConsoleCommand::Start));
        assert_eq!(parse_console(b"!logger-stop"), Some(ConsoleCommand::Stop));
        assert_eq!(
            parse_console(b"logger config 250"),
            Some(ConsoleCommand::Config(Config {
                sync_period_ms: 250
            }))
        );
        assert_eq!(parse_console(b"logger config bad"), None);
        assert!(accepts(Status::Disabled, Command::Start));
        assert!(accepts(Status::Error, Command::Start));
        assert!(accepts(Status::Ready, Command::Stop));
        assert!(accepts(Status::Ready, Command::Sync));
        assert!(!accepts(Status::Initializing, Command::Stop));
        assert!(!accepts(Status::Stopping, Command::Start));
    }

    #[test]
    fn log_ranges_and_names_are_bounded() {
        assert!(is_ulog_name(b"00000000.ULG"));
        assert!(is_ulog_name(b"A.ulg"));
        assert!(!is_ulog_name(b"00000000.CSV"));
        assert_eq!(list_bounds(3, 0, u16::MAX), Some((0, 2)));
        assert_eq!(list_bounds(3, 2, 1), None);
        assert_eq!(list_bounds(3, 3, u16::MAX), None);
        assert_eq!(download_count(100, 10, 20), Ok(20));
        assert_eq!(download_count(100, 90, u32::MAX), Ok(10));
        assert_eq!(download_count(100, 100, 1), Err(LogError::Range));
    }
}
