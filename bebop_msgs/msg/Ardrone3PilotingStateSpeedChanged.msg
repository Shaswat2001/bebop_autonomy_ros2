# Ardrone3PilotingStateSpeedChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Drones speed changed. Expressed in the NED referential (North-East-Down).

std_msgs/Header header

# Speed relative to the North (when drone moves to the north, speed is &gt; 0) (in m/s)
float32 speed_x
# Speed relative to the East (when drone moves to the east, speed is &gt; 0) (in m/s)
float32 speed_y
# Speed on the z axis (when drone moves down, speed is &gt; 0) (in m/s)
float32 speed_z
