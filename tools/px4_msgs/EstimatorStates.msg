# Runtime-field extract: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4
# msg/EstimatorStates.msg; BSD-3-Clause, see LICENSE.
uint64 timestamp
uint64 timestamp_sample
float32[25] states
uint8 n_states
float32[24] covariances
