//! HIGHRES_IMU magnetic fields from the current board-FRD BMM150 sample.

use mavlink::common::{HighresImuUpdatedFlags, HIGHRES_IMU_DATA};

use crate::px4_messages::VehicleMagnetometer;

const MAG_UPDATED: HighresImuUpdatedFlags = HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XMAG
    .union(HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YMAG)
    .union(HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZMAG);

pub fn update_highres_imu(
    data: &mut HIGHRES_IMU_DATA,
    sample: Option<VehicleMagnetometer>,
    now: u64,
) {
    data.fields_updated.remove(MAG_UPDATED);
    let field = sample
        .filter(|value| value.fresh(now))
        .map_or([f32::NAN; 3], |value| {
            data.fields_updated |= MAG_UPDATED;
            value.magnetometer_ga
        });
    [data.xmag, data.ymag, data.zmag] = field;
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::{common::MavMessage, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message};

    #[test]
    fn fresh_field_is_gauss_and_bad_samples_clear_only_the_magnetic_updated_bits() {
        let sample = VehicleMagnetometer {
            timestamp: 10_000,
            timestamp_sample: 10_000,
            device_id: crate::bmm150::DEVICE_ID,
            magnetometer_ga: [0.22, -0.07, 0.43],
            calibration_count: 0,
        };
        let mut data = HIGHRES_IMU_DATA {
            fields_updated: HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC,
            ..Default::default()
        };
        update_highres_imu(&mut data, Some(sample), 10_000);
        assert_eq!([data.xmag, data.ymag, data.zmag], sample.magnetometer_ga);
        assert!(data.fields_updated.contains(MAG_UPDATED));
        let message = MavMessage::HIGHRES_IMU(data.clone());
        let mut frame = MAVLinkV2MessageRaw::new();
        frame.serialize_message(
            MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            },
            &message,
        );
        assert!(frame.has_valid_crc::<MavMessage>());
        assert_eq!(
            MavMessage::parse(MavlinkVersion::V2, frame.message_id(), frame.payload()).unwrap(),
            message
        );
        for (sample, now) in [
            (None, 10_000),
            (Some(sample), 9_999),
            (Some(sample), 10_001 + crate::bmm150::FRESH_US),
        ] {
            update_highres_imu(&mut data, sample, now);
            assert!([data.xmag, data.ymag, data.zmag].iter().all(|v| v.is_nan()));
            assert_eq!(
                data.fields_updated,
                HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
            );
        }
    }
}
