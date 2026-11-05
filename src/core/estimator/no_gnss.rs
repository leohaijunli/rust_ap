//! Independent EKF: the typed input API excludes GNSS; NavFilter ignores it.

use super::{
    imu_ekf::{
        baro_mag::{BaroMagParams, BaroMagSample},
        range::RangeParams,
        ImuEkf, ImuEkfParams,
    },
    output_predictor::OutputParams,
    range_sensor::RangeObservation,
    FilterFault, ImuDelta,
};

/// Barometer, magnetometer and range-aided estimator for local dead reckoning.
///
/// The typed API rejects GNSS, and the common NavFilter API discards it before
/// touching filter state. Unconstrained horizontal navigation remains invalid
/// and uses the dedicated odometry stream with unbounded uncertainty.
pub struct NoGnssEkf {
    core: ImuEkf,
}

impl NoGnssEkf {
    pub(super) const fn dormant() -> Self {
        Self {
            core: ImuEkf::dormant(),
        }
    }

    pub fn reconfigure(
        &mut self,
        imu: ImuEkfParams,
        baro: BaroMagParams,
        range: RangeParams,
        t0: u64,
    ) -> Result<(), FilterFault> {
        super::config_stage::enter(super::config_stage::IMU_INIT);
        self.core.initialize_range(&imu, baro, range, t0)
    }

    pub fn new(
        imu_params: ImuEkfParams,
        baro_mag_params: BaroMagParams,
        range_params: RangeParams,
        t0: u64,
    ) -> Result<Self, FilterFault> {
        Ok(Self {
            core: ImuEkf::with_range(&imu_params, baro_mag_params, range_params, t0)?,
        })
    }

    pub fn reset(&mut self, t0: u64) -> Result<(), FilterFault> {
        self.core.reset(t0)
    }

    pub fn configure_output(&mut self, params: OutputParams) -> Result<(), FilterFault> {
        self.core.configure_output(params)
    }

    pub fn debug_core(&self) -> &ImuEkf {
        &self.core
    }

    pub const fn core(&self) -> &ImuEkf {
        &self.core
    }

    pub fn standard_output(&mut self, now: u64) -> super::standard_output::Snapshot {
        super::standard_output::Snapshot::from_core(
            &mut self.core,
            now,
            super::standard_output::Source::NoGnss,
            None,
        )
    }

    /// The range input cannot be substituted with a GNSS observation.
    ///
    /// ```compile_fail
    /// use v6x_rust_led::estimator::{gnss::GnssObservation, no_gnss::NoGnssEkf, ImuDelta};
    /// fn inject_gnss(filter: &mut NoGnssEkf, imu: &ImuDelta, sample: GnssObservation) {
    ///     filter.update(imu, None, Some(sample));
    /// }
    /// ```
    pub fn update(
        &mut self,
        imu: &ImuDelta,
        baro_mag: Option<BaroMagSample>,
        range: Option<RangeObservation>,
    ) -> Result<bool, FilterFault> {
        self.core.update_range(imu, baro_mag, range)
    }

    pub fn update_baro(
        &mut self,
        imu: &ImuDelta,
        baro: Option<super::imu_ekf::baro_mag::BaroAltitude>,
        range: Option<RangeObservation>,
    ) -> Result<bool, FilterFault> {
        self.core.update_baro(imu, baro, range)
    }
}

impl super::NavFilter for NoGnssEkf {
    type Params = super::runtime_params::Params;

    fn new(params: &Self::Params, t0: u64) -> Self {
        let mut filter = const { Self::dormant() };
        let _ = filter.configure_from_snapshot(params, t0); // Stored in health().
        filter
    }

    fn push_imu(&mut self, sample: &ImuDelta) {
        let _ = self.core.ingest_imu(sample); // Failure remains in health().
    }

    fn push_measurement(&mut self, measurement: super::Measurement) {
        // The common trait can carry GNSS, but this instance never consumes it.
        // Its typed legacy API also continues to reject GNSS at compile time.
        if matches!(measurement, super::Measurement::Gnss(_)) || self.core.health().fault.is_some()
        {
            return;
        }
        if let Err(fault) = self.core.enqueue(measurement) {
            self.core.invalidate(fault);
        }
    }

    fn step(&mut self, now: u64) -> super::StepReport {
        let before = self.all_aid_status();
        let result = self.core.check_fresh(now).and_then(|()| self.core.step());
        match result {
            Ok(updated) => super::StepReport::from_aids(updated, before, self.all_aid_status()),
            Err(fault) => {
                self.core.invalidate(fault);
                super::StepReport::default()
            }
        }
    }

    fn output(&self) -> super::NavOutput {
        self.core.output()
    }
    fn health(&self) -> super::FilterHealth {
        self.core.health()
    }
}

impl NoGnssEkf {
    #[inline(never)]
    pub fn configure_from_snapshot(
        &mut self,
        params: &super::runtime_params::Params,
        t0: u64,
    ) -> Result<(), FilterFault> {
        super::config_stage::enter(super::config_stage::VALIDATE);
        let result = params
            .validate(false)
            .and_then(|()| self.reconfigure(params.imu, params.aiding, params.range, t0))
            .and_then(|()| {
                super::config_stage::enter(super::config_stage::OUTPUT);
                self.configure_output(params.output)
            });
        if let Err(fault) = result {
            self.core.invalidate(fault);
        }
        result
    }

    fn all_aid_status(&self) -> [super::imu_ekf::baro_mag::AidStatus; 3] {
        let (baro, mag) = self.core.aid_status().unwrap_or_default();
        [baro, mag, self.core.range_status().unwrap_or_default()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::{
        gnss::{GnssEkf, GnssObservation, GnssParams},
        GnssSample,
    };

    const DT_S: f32 = 0.0025;
    const DT_US: u64 = 2_500;

    #[test]
    fn corrupt_gnss_in_the_main_instance_cannot_change_the_independent_instance() {
        let imu_params = ImuEkfParams::default();
        let aiding_params = BaroMagParams::default();
        let range_params = RangeParams::default();
        let mut main = GnssEkf::new(
            imu_params,
            aiding_params,
            range_params,
            GnssParams::default(),
            0,
        )
        .unwrap();
        let mut independent = NoGnssEkf::new(imu_params, aiding_params, range_params, 0).unwrap();
        let mut baseline = NoGnssEkf::new(imu_params, aiding_params, range_params, 0).unwrap();
        let mut main_fault = false;
        for index in 1..=1_600_u64 {
            let timestamp = index * DT_US;
            let imu = ImuDelta {
                timestamp,
                delta_angle_rad: [0.0; 3],
                delta_velocity_m_s: [0.0, 0.0, -super::super::math::GRAVITY * DT_S],
                delta_angle_dt_s: DT_S,
                delta_velocity_dt_s: DT_S,
                delta_velocity_clipping: [false; 3],
            };
            let aiding = index.is_multiple_of(8).then_some(BaroMagSample {
                timestamp,
                altitude_m: 0.0,
                field_gauss: [0.22, 0.0, 0.43],
            });
            if timestamp == 2_000_000 {
                main_fault = main
                    .update(
                        &imu,
                        aiding,
                        Some(GnssObservation {
                            sample: GnssSample {
                                timestamp,
                                latitude_deg: f64::NAN,
                                ..GnssSample::default()
                            },
                            pdop: 1.0,
                            spoofed: false,
                        }),
                        None,
                    )
                    .is_err();
            } else if !main_fault {
                main.update(&imu, aiding, None, None).unwrap();
            }
            independent.update(&imu, aiding, None).unwrap();
            baseline.update(&imu, aiding, None).unwrap();
            assert_eq!(independent.core().state(), baseline.core().state());
            assert_eq!(
                independent.core().covariance(),
                baseline.core().covariance()
            );
            assert_eq!(independent.core().output(), baseline.core().output());
        }
        assert!(main_fault);
        assert_eq!(main.core().health().fault, Some(FilterFault::Input));
        assert_eq!(independent.core().health().fault, None);
    }

    #[test]
    fn sixty_second_acceleration_bias_remains_local_dead_reckoning() {
        let mut estimator = NoGnssEkf::new(
            ImuEkfParams::default(),
            BaroMagParams::default(),
            RangeParams::default(),
            0,
        )
        .unwrap();

        for index in 1..=24_000_u64 {
            let timestamp = index * DT_US;
            let aiding = (index % 8 == 0).then_some(BaroMagSample {
                timestamp,
                altitude_m: 0.0,
                field_gauss: [0.22, 0.0, 0.43],
            });
            let range = (index % 20 == 0).then_some(RangeObservation {
                timestamp,
                distance_m: 1.0,
                quality_percent: 100,
                limits: None,
            });
            estimator
                .update(
                    &ImuDelta {
                        timestamp,
                        delta_angle_rad: [0.0; 3],
                        delta_velocity_m_s: [
                            if timestamp > 5_000_000 {
                                0.05 * DT_S
                            } else {
                                0.0
                            },
                            0.0,
                            -super::super::math::GRAVITY * DT_S,
                        ],
                        delta_angle_dt_s: DT_S,
                        delta_velocity_dt_s: DT_S,
                        delta_velocity_clipping: [false; 3],
                    },
                    aiding,
                    range,
                )
                .unwrap();
        }

        let core = estimator.core();
        let output = core.output();
        assert_eq!(
            core.control_status() & ((1 << 2) | (1 << 11) | (1 << 44)),
            0
        );
        assert_ne!(core.control_status() & (1 << 29), 0);
        assert!(output.position_ned_m.iter().all(|value| value.is_finite()));
        assert!(output
            .velocity_ned_m_s
            .iter()
            .all(|value| value.is_finite()));
        assert!(
            output.position_ned_m[0].abs() > 0.01,
            "position={:?} velocity={:?} covariance_xy=({}, {})",
            output.position_ned_m,
            output.velocity_ned_m_s,
            core.covariance()[6][6],
            core.covariance()[7][7],
        );
        assert!(core.origin.is_none());
        let snapshot = estimator.standard_output(60_000_000);
        assert!(!snapshot.local_position.xy_valid);
        assert!(!snapshot.local_position.v_xy_valid);
        assert!(snapshot.global_position.is_none());
        let odometry = snapshot.odometry.unwrap();
        assert_eq!(&odometry.position_variance[..2], &[f32::INFINITY; 2]);
        assert_eq!(&odometry.velocity_variance[..2], &[f32::INFINITY; 2]);
    }
}
