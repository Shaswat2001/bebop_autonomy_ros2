# Ardrone3MediaRecordStateVideoResolutionState
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Video resolution. Informs about streaming and recording video resolutions. Note that this is only an indication about what the resolution should be. To know the real resolution, you should get it from the frame.

std_msgs/Header header

# Streaming resolution
uint8 STREAMING_RES360P=0  # 360p resolution.
uint8 STREAMING_RES480P=1  # 480p resolution.
uint8 STREAMING_RES720P=2  # 720p resolution.
uint8 STREAMING_RES1080P=3  # 1080p resolution.
uint8 streaming
# Recording resolution
uint8 RECORDING_RES360P=0  # 360p resolution.
uint8 RECORDING_RES480P=1  # 480p resolution.
uint8 RECORDING_RES720P=2  # 720p resolution.
uint8 RECORDING_RES1080P=3  # 1080p resolution.
uint8 recording
