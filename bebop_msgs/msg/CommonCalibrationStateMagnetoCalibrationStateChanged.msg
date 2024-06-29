# CommonCalibrationStateMagnetoCalibrationStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Magneto calib process axis state.

Header header

# State of the x axis (roll) calibration : 1 if calibration is done, 0 otherwise
uint8 x_axis_calibration
# State of the y axis (pitch) calibration : 1 if calibration is done, 0 otherwise
uint8 y_axis_calibration
# State of the z axis (yaw) calibration : 1 if calibration is done, 0 otherwise
uint8 z_axis_calibration
# 1 if calibration has failed, 0 otherwise. If this arg is 1, consider all previous arg as 0
uint8 calibration_failed
