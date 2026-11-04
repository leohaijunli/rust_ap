# Runtime-field extract: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4
# msg/EstimatorAidSource1d.msg; BSD-3-Clause, see LICENSE.
uint64 timestamp
uint64 timestamp_sample
uint8 estimator_instance
uint32 device_id
uint64 time_last_fuse
float32 observation
float32 observation_variance
float32 innovation
float32 innovation_filtered
float32 innovation_variance
float32 test_ratio
float32 test_ratio_filtered
bool innovation_rejected
bool fused
