# CommonCommonStateMassStorageContentForCurrentRun
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Mass storage content for current run. Only counts the files related to the current run (see [RunId](#0-30-0))

std_msgs/Header header

# Mass storage id (unique)
uint8 mass_storage_id
# Number of photos (does not include raw photos)
uint16 nb_photos
# Number of videos
uint16 nb_videos
# Number of raw photos
uint16 nb_raw_photos
