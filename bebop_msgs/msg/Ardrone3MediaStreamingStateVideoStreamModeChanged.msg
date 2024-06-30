# Ardrone3MediaStreamingStateVideoStreamModeChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: 

std_msgs/Header header

# stream mode
uint8 MODE_LOW_LATENCY=0  # Minimize latency with average reliability (best for piloting).
uint8 MODE_HIGH_RELIABILITY=1  # Maximize the reliability with an average latency (best when streaming quality is important but not the latency).
uint8 MODE_HIGH_RELIABILITY_LOW_FRAMERATE=2  # Maximize the reliability using a framerate decimation with an average latency (best when streaming quality is important but not the latency).
uint8 mode
