# SPDX-License-Identifier: Apache-2.0
# Receipt-time command contract. valid_for begins when the adapter callback
# receives this message; no sender timestamp is trusted for the watchdog.
uint64 sequence_id
float64 linear_speed_mps
float64 yaw_rate_radps
duration valid_for
