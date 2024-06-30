# Ardrone3PilotingStateFlyingStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Flying state.

std_msgs/Header header

# Drone flying state
uint8 STATE_LANDED=0  # Landed state
uint8 STATE_TAKINGOFF=1  # Taking off state
uint8 STATE_HOVERING=2  # Hovering / Circling (for fixed wings) state
uint8 STATE_FLYING=3  # Flying state
uint8 STATE_LANDING=4  # Landing state
uint8 STATE_EMERGENCY=5  # Emergency state
uint8 STATE_USERTAKEOFF=6  # User take off state. Waiting for user action to take off.
uint8 STATE_MOTOR_RAMPING=7  # Motor ramping state (for fixed wings).
uint8 STATE_EMERGENCY_LANDING=8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.
uint8 state
