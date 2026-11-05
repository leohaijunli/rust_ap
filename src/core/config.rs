//! Host-testable configuration API between parameters, drivers and algorithms.

use crate::icm42688::{Config as ImuConfig, ConfigError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Mahony,
    Ekf2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    imu: ImuConfig,
    algorithm: Algorithm,
}

impl Config {
    pub const DEFAULT: Self = Self {
        imu: ImuConfig::DEFAULT,
        algorithm: Algorithm::Mahony,
    };

    pub const fn new(imu: ImuConfig, algorithm: Algorithm) -> Result<Self, ConfigError> {
        match imu.validate() {
            Ok(imu) => Ok(Self { imu, algorithm }),
            Err(error) => Err(error),
        }
    }

    pub const fn imu(self) -> ImuConfig {
        self.imu
    }

    pub const fn algorithm(self) -> Algorithm {
        self.algorithm
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_sdk_configuration_is_valid_and_explicit() {
        let config = Config::DEFAULT;
        assert_eq!(config.imu().validate(), Ok(config.imu()));
        assert_eq!(config.algorithm(), Algorithm::Mahony);
    }

    #[test]
    fn invalid_driver_configuration_never_crosses_the_api_boundary() {
        let invalid = ImuConfig {
            odr_hz: 4_000,
            ..ImuConfig::DEFAULT
        };
        assert_eq!(
            Config::new(invalid, Algorithm::Mahony),
            Err(ConfigError::Odr)
        );
    }
}
