# CommonMavlinkStateMavlinkPlayErrorStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: FlightPlan error.

std_msgs/Header header

# State of play error
uint8 ERROR_NONE=0  # There is no error
uint8 ERROR_NOTINOUTDOORMODE=1  # The drone is not in outdoor mode
uint8 ERROR_GPSNOTFIXED=2  # The gps is not fixed
uint8 ERROR_NOTCALIBRATED=3  # The magnetometer of the drone is not calibrated
uint8 error
