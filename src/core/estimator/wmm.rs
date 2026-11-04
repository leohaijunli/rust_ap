//! Fixed PX4 WMM bilinear lookup; no online model update or heap.
//! Copyright (c) 2014-2024 PX4 Development Team. All rights reserved.
//! SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE.

use super::{wmm_generated as tables, FilterFault};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagneticField {
    pub declination_deg: f32,
    pub inclination_deg: f32,
    pub strength_gauss: f32,
}

fn interpolate(latitude: f32, mut longitude: f32, table: &[[i16; 37]; 19]) -> f32 {
    let latitude = latitude.clamp(-90.0, 90.0);
    if longitude > 180.0 {
        longitude -= 360.0;
    }
    if longitude < -180.0 {
        longitude += 360.0;
    }
    let low_lat = (libm::floorf(latitude / 10.0) * 10.0).clamp(-90.0, 80.0);
    let low_lon = (libm::floorf(longitude / 10.0) * 10.0).clamp(-180.0, 170.0);
    let i = ((90.0 + low_lat) / 10.0) as usize;
    let j = ((180.0 + low_lon) / 10.0) as usize;
    let (sw, se, ne, nw) = (
        f32::from(table[i][j]),
        f32::from(table[i][j + 1]),
        f32::from(table[i + 1][j + 1]),
        f32::from(table[i + 1][j]),
    );
    let lat_scale = ((latitude - low_lat) / 10.0).clamp(0.0, 1.0);
    let lon_scale = ((longitude - low_lon) / 10.0).clamp(0.0, 1.0);
    let low = lon_scale * (se - sw) + sw;
    let high = lon_scale * (ne - nw) + nw;
    lat_scale * (high - low) + low
}

pub fn lookup(latitude_deg: f32, longitude_deg: f32) -> Result<MagneticField, FilterFault> {
    if !latitude_deg.is_finite() || !longitude_deg.is_finite() {
        return Err(FilterFault::Input);
    }
    Ok(MagneticField {
        declination_deg: interpolate(latitude_deg, longitude_deg, &tables::DECLINATION)
            * tables::DECLINATION_SCALE,
        inclination_deg: interpolate(latitude_deg, longitude_deg, &tables::INCLINATION)
            * tables::INCLINATION_SCALE,
        strength_gauss: interpolate(latitude_deg, longitude_deg, &tables::TOTALINTENSITY)
            * tables::TOTALINTENSITY_SCALE
            * 1e-9
            * 1e4,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn globe_edges_interpolate_without_out_of_bounds_and_wrap_once() {
        for lat in [-90.0, -89.9, 0.0, 47.397742, 90.0] {
            for lon in [-180.0, 8.545594, 179.99, 180.0] {
                let result = lookup(lat, lon).unwrap();
                assert!(result.declination_deg.is_finite());
                assert!(result.inclination_deg.is_finite());
                assert!(result.strength_gauss > 0.0);
            }
        }
        assert_eq!(lookup(0.0, 190.0), lookup(0.0, -170.0));
        assert_eq!(lookup(f32::NAN, 0.0), Err(FilterFault::Input));
    }
}
