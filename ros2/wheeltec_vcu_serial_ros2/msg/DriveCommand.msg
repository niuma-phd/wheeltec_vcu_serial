# SPDX-License-Identifier: Apache-2.0
# The adapter assigns freshness at callback receipt. valid_for is measured from
# that local monotonic receipt; it is not a source timestamp or network budget.
uint64 sequence_id
float64 linear_speed_mps
float64 yaw_rate_radps
builtin_interfaces/Duration valid_for
