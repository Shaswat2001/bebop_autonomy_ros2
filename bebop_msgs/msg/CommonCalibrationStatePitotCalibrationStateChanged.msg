# CommonCalibrationStatePitotCalibrationStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: 

std_msgs/Header header

# State of pitot calibration
uint8 STATE_DONE=0  # Calibration is ok
uint8 STATE_READY=1  # Calibration is started, waiting user action
uint8 STATE_IN_PROGRESS=2  # Calibration is in progress
uint8 STATE_REQUIRED=3  # Calibration is required
uint8 state
# lastError : 1 if an error occured and 0 if not
uint8 last_error
