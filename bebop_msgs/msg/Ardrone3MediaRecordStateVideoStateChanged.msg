# Ardrone3MediaRecordStateVideoStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Picture record state.

Header header

# State of video
uint8 STATE_STOPPED=0  # Video was stopped
uint8 STATE_STARTED=1  # Video was started
uint8 STATE_FAILED=2  # Video was failed
uint8 STATE_AUTOSTOPPED=3  # Video was auto stopped
uint8 state
# Mass storage id where the video was recorded
uint8 mass_storage_id
