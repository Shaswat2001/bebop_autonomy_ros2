# CommonChargerStateLastChargeRateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Last charge rate.

std_msgs/Header header

# The charge rate recorded by the firmware for the last charge.
uint8 RATE_UNKNOWN=0  # The last charge rate is not known.
uint8 RATE_SLOW=1  # Slow charge rate.
uint8 RATE_MODERATE=2  # Moderate charge rate.
uint8 RATE_FAST=3  # Fast charge rate.
uint8 rate
