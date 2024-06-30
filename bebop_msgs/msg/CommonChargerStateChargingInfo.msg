# CommonChargerStateChargingInfo
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Charging information.

std_msgs/Header header

# The current charging phase.
uint8 PHASE_UNKNOWN=0  # The charge phase is unknown or irrelevant.
uint8 PHASE_CONSTANT_CURRENT_1=1  # First phase of the charging process. The battery is charging with constant current.
uint8 PHASE_CONSTANT_CURRENT_2=2  # Second phase of the charging process. The battery is charging with constant current, with a higher voltage than the first phase.
uint8 PHASE_CONSTANT_VOLTAGE=3  # Last part of the charging process. The battery is charging with a constant voltage.
uint8 PHASE_CHARGED=4  # The battery is fully charged.
uint8 PHASE_DISCHARGING=5  # The battery is discharging; Other arguments refers to the last charge.
uint8 phase
# The charge rate. If phase is DISCHARGING, refers to the last charge.
uint8 RATE_UNKNOWN=0  # The charge rate is not known.
uint8 RATE_SLOW=1  # Slow charge rate.
uint8 RATE_MODERATE=2  # Moderate charge rate.
uint8 RATE_FAST=3  # Fast charge rate.
uint8 rate
# The charging intensity, in dA. (12dA = 1,2A) ; If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 intensity
# The full charging time estimated, in minute. If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 full_charging_time
