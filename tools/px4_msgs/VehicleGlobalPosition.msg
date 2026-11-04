# Runtime-field extract: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4
# msg/versioned/VehicleGlobalPosition.msg; BSD-3-Clause, see LICENSE.
uint64 timestamp
uint64 timestamp_sample
float64 lat
float64 lon
float32 alt
float32 alt_ellipsoid
bool lat_lon_valid
bool alt_valid
float32 delta_alt
float32 delta_terrain
uint8 lat_lon_reset_counter
uint8 alt_reset_counter
uint8 terrain_reset_counter
float32 eph
float32 epv
float32 terrain_alt
bool terrain_alt_valid
bool dead_reckoning
