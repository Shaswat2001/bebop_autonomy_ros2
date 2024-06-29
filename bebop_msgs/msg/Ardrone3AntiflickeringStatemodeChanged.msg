# Ardrone3AntiflickeringStatemodeChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Antiflickering mode.

Header header

# Mode of the anti flickering functionnality
uint8 MODE_AUTO=0  # Anti flickering based on the electric frequency previously sent
uint8 MODE_FIXEDFIFTYHERTZ=1  # Anti flickering based on a fixed frequency of 50Hz
uint8 MODE_FIXEDSIXTYHERTZ=2  # Anti flickering based on a fixed frequency of 60Hz
uint8 mode
