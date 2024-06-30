# Ardrone3PilotingStateAlertStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Alert state.

std_msgs/Header header

# Drone alert state
uint8 STATE_NONE=0  # No alert
uint8 STATE_USER=1  # User emergency alert
uint8 STATE_CUT_OUT=2  # Cut out alert
uint8 STATE_CRITICAL_BATTERY=3  # Critical battery alert
uint8 STATE_LOW_BATTERY=4  # Low battery alert
uint8 STATE_TOO_MUCH_ANGLE=5  # The angle of the drone is too high
uint8 state
