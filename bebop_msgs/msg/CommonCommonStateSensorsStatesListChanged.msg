# CommonCommonStateSensorsStatesListChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Sensors state list.

std_msgs/Header header

# Sensor name
uint8 SENSORNAME_IMU=0  # Inertial Measurement Unit sensor
uint8 SENSORNAME_BAROMETER=1  # Barometer sensor
uint8 SENSORNAME_ULTRASOUND=2  # Ultrasonic sensor
uint8 SENSORNAME_GPS=3  # GPS sensor
uint8 SENSORNAME_MAGNETOMETER=4  # Magnetometer sensor
uint8 SENSORNAME_VERTICAL_CAMERA=5  # Vertical Camera sensor
uint8 sensor_name
# Sensor state (1 if the sensor is OK, 0 if the sensor is NOT OK)
uint8 sensor_state
