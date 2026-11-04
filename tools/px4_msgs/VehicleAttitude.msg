# Runtime-field extract: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4
# msg/versioned/VehicleAttitude.msg; BSD-3-Clause, see LICENSE.
uint64 timestamp
uint64 timestamp_sample
float32[4] q
float32[4] delta_q_reset
uint8 quat_reset_counter
