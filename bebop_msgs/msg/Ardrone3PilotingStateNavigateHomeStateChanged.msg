# Ardrone3PilotingStateNavigateHomeStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Return home state. Availability is related to gps fix, magnetometer calibration.

std_msgs/Header header

# State of navigate home
uint8 STATE_AVAILABLE=0  # Navigate home is available
uint8 STATE_INPROGRESS=1  # Navigate home is in progress
uint8 STATE_UNAVAILABLE=2  # Navigate home is not available
uint8 STATE_PENDING=3  # Navigate home has been received, but its process is pending
uint8 state
# Reason of the state
uint8 REASON_USERREQUEST=0  # User requested a navigate home (available-&gt;inProgress)
uint8 REASON_CONNECTIONLOST=1  # Connection between controller and product lost (available-&gt;inProgress)
uint8 REASON_LOWBATTERY=2  # Low battery occurred (available-&gt;inProgress)
uint8 REASON_FINISHED=3  # Navigate home is finished (inProgress-&gt;available)
uint8 REASON_STOPPED=4  # Navigate home has been stopped (inProgress-&gt;available)
uint8 REASON_DISABLED=5  # Navigate home disabled by product (inProgress-&gt;unavailable or available-&gt;unavailable)
uint8 REASON_ENABLED=6  # Navigate home enabled by product (unavailable-&gt;available)
uint8 reason
