# Ardrone3PilotingStateLandingStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Landing state. Only available for fixed wings (which have two landing modes).

std_msgs/Header header

# Drone landing state
uint8 STATE_LINEAR=0  # Linear landing
uint8 STATE_SPIRAL=1  # Spiral landing
uint8 state
