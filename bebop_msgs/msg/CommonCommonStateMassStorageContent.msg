# CommonCommonStateMassStorageContent
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Mass storage content.

std_msgs/Header header

# Mass storage id (unique)
uint8 mass_storage_id
# Number of photos (does not include raw photos)
uint16 nb_photos
# Number of videos
uint16 nb_videos
# Number of puds
uint16 nb_puds
# Number of crash logs
uint16 nb_crash_logs
# Number of raw photos
uint16 nb_raw_photos
