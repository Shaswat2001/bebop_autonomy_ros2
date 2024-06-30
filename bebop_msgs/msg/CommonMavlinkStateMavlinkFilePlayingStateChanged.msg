# CommonMavlinkStateMavlinkFilePlayingStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Playing state of a FlightPlan.

std_msgs/Header header

# State of the mavlink
uint8 STATE_PLAYING=0  # Mavlink file is playing
uint8 STATE_STOPPED=1  # Mavlink file is stopped (arg filepath and type are useless in this state)
uint8 STATE_PAUSED=2  # Mavlink file is paused
uint8 STATE_LOADED=3  # Mavlink file is loaded (it will be played at take-off)
uint8 state
# flight plan file path from the mavlink ftp root
string filepath
# type of the played mavlink file
uint8 TYPE_FLIGHTPLAN=0  # Mavlink file for FlightPlan
uint8 TYPE_MAPMYHOUSE=1  # Mavlink file for MapMyHouse
uint8 type
