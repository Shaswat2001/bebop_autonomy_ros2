# Ardrone3MediaRecordStatePictureStateChangedV2
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Picture state.

std_msgs/Header header

# State of device picture recording
uint8 STATE_READY=0  # The picture recording is ready
uint8 STATE_BUSY=1  # The picture recording is busy
uint8 STATE_NOTAVAILABLE=2  # The picture recording is not available
uint8 state
# Error to explain the state
uint8 ERROR_OK=0  # No Error
uint8 ERROR_UNKNOWN=1  # Unknown generic error
uint8 ERROR_CAMERA_KO=2  # Picture camera is out of order
uint8 ERROR_MEMORYFULL=3  # Memory full ; cannot save one additional picture
uint8 ERROR_LOWBATTERY=4  # Battery is too low to start/keep recording.
uint8 error
