#ifndef BEBOP_DRIVER_HPP
#define BEBOP_DRIVER_HPP

namespace bebop_driver {
class BebopArdrone3Config {
public:
  BebopArdrone3Config(double PilotingSettingsMaxAltitudeCurrent_,
                      double PilotingSettingsMaxTiltCurrent_,
                      int PilotingSettingsAbsolutControlOn_,
                      double PilotingSettingsMaxDistanceValue_,
                      int PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_,
                      int PilotingSettingsBankedTurnValue_,
                      double PilotingSettingsMinAltitudeCurrent_,
                      int PilotingSettingsCirclingDirectionValue_,
                      int PilotingSettingsCirclingRadiusValue_,
                      int PilotingSettingsCirclingAltitudeValue_,
                      int PilotingSettingsPitchModeValue_,
                      double SpeedSettingsMaxVerticalSpeedCurrent_,
                      double SpeedSettingsMaxRotationSpeedCurrent_,
                      int SpeedSettingsHullProtectionPresent_,
                      int SpeedSettingsOutdoorOutdoor_,
                      double SpeedSettingsMaxPitchRollRotationSpeedCurrent_,
                      int NetworkSettingsWifiSelectionType_,
                      int NetworkSettingsWifiSelectionBand_,
                      int NetworkSettingsWifiSelectionChannel_,
                      int PictureSettingsVideoStabilizationModeMode_,
                      int PictureSettingsVideoRecordingModeMode_,
                      int PictureSettingsVideoFramerateFramerate_,
                      int PictureSettingsVideoResolutionsType_,
                      int gpsSettingsHomeTypeType_,
                      int gpsSettingsReturnHomeDelayDelay_)
    : PilotingSettingsMaxAltitudeCurrent(PilotingSettingsMaxAltitudeCurrent_),
      PilotingSettingsMaxTiltCurrent(PilotingSettingsMaxTiltCurrent_),
      PilotingSettingsAbsolutControlOn(PilotingSettingsAbsolutControlOn_),
      PilotingSettingsMaxDistanceValue(PilotingSettingsMaxDistanceValue_),
      PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover(PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_),
      PilotingSettingsBankedTurnValue(PilotingSettingsBankedTurnValue_),
      PilotingSettingsMinAltitudeCurrent(PilotingSettingsMinAltitudeCurrent_),
      PilotingSettingsCirclingDirectionValue(PilotingSettingsCirclingDirectionValue_),
      PilotingSettingsCirclingRadiusValue(PilotingSettingsCirclingRadiusValue_),
      PilotingSettingsCirclingAltitudeValue(PilotingSettingsCirclingAltitudeValue_),
      PilotingSettingsPitchModeValue(PilotingSettingsPitchModeValue_),
      SpeedSettingsMaxVerticalSpeedCurrent(SpeedSettingsMaxVerticalSpeedCurrent_),
      SpeedSettingsMaxRotationSpeedCurrent(SpeedSettingsMaxRotationSpeedCurrent_),
      SpeedSettingsHullProtectionPresent(SpeedSettingsHullProtectionPresent_),
      SpeedSettingsOutdoorOutdoor(SpeedSettingsOutdoorOutdoor_),
      SpeedSettingsMaxPitchRollRotationSpeedCurrent(SpeedSettingsMaxPitchRollRotationSpeedCurrent_),
      NetworkSettingsWifiSelectionType(NetworkSettingsWifiSelectionType_),
      NetworkSettingsWifiSelectionBand(NetworkSettingsWifiSelectionBand_),
      NetworkSettingsWifiSelectionChannel(NetworkSettingsWifiSelectionChannel_),
      PictureSettingsVideoStabilizationModeMode(PictureSettingsVideoStabilizationModeMode_),
      PictureSettingsVideoRecordingModeMode(PictureSettingsVideoRecordingModeMode_),
      PictureSettingsVideoFramerateFramerate(PictureSettingsVideoFramerateFramerate_),
      PictureSettingsVideoResolutionsType(PictureSettingsVideoResolutionsType_),
      GPSSettingsHomeTypeType(gpsSettingsHomeTypeType_),
      GPSSettingsReturnHomeDelayDelay(gpsSettingsReturnHomeDelayDelay_) {}

  double PilotingSettingsMaxAltitudeCurrent;
  double PilotingSettingsMaxTiltCurrent;
  int PilotingSettingsAbsolutControlOn;
  double PilotingSettingsMaxDistanceValue;
  int PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover;
  int PilotingSettingsBankedTurnValue;
  double PilotingSettingsMinAltitudeCurrent;
  int PilotingSettingsCirclingDirectionValue;
  int PilotingSettingsCirclingRadiusValue;
  int PilotingSettingsCirclingAltitudeValue;
  int PilotingSettingsPitchModeValue;
  double SpeedSettingsMaxVerticalSpeedCurrent;
  double SpeedSettingsMaxRotationSpeedCurrent;
  int SpeedSettingsHullProtectionPresent;
  int SpeedSettingsOutdoorOutdoor;
  double SpeedSettingsMaxPitchRollRotationSpeedCurrent;
  int NetworkSettingsWifiSelectionType;
  int NetworkSettingsWifiSelectionBand;
  int NetworkSettingsWifiSelectionChannel;
  int PictureSettingsVideoStabilizationModeMode;
  int PictureSettingsVideoRecordingModeMode;
  int PictureSettingsVideoFramerateFramerate;
  int PictureSettingsVideoResolutionsType;
  int GPSSettingsHomeTypeType;
  int GPSSettingsReturnHomeDelayDelay;
};

}

#endif // BEBOP_DRIVER_HPP