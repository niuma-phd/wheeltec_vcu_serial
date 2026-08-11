# SPDX-License-Identifier: Apache-2.0
# receipt_time is the ROS clock sampled after a validated frame is read.
# The controller provides no source timestamp or command acknowledgement.
time receipt_time
uint8 composite_stop_flag_raw
bool control_allowed
bool control_inhibited
float64 linear_speed_mps
float64 lateral_speed_mps
float64 yaw_rate_radps
float64[3] linear_acceleration_mps2
float64[3] angular_velocity_radps
float64 supply_voltage_v
bool vcu_ack_available
bool source_time_available
