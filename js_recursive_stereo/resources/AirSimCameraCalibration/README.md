These images have been created with AirSim and a edited version of the Blocks Environment with a 8x7 checkerboard pattern with 1000mm x 1000mm square size (and thus a size of 8000mm x 7000mm for the entire checker board).


The following settings in settings.json were used:

{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingdVersion": 1.0,
  "PhysicsEngineName": "FastPhysicsEngine",
  "CaptureSettings": [
    {
      "ImageType": 0,
      "Width": 800,
      "Height": 600,
      "FOV_Degrees": 70,
      "AutoExposureSpeed": 100,
      "MotionBlurAmount": 0
    },
    {
      "ImageType": 1,
      "Width": 800,
      "Height": 600,
      "FOV_Degrees": 70,
      "AutoExposureSpeed": 100,
      "MotionBlurAmount": 0
    }
  ],
  "UsageScenario": "ComputerVision",
  "SimpleFlight": {
    "FirmwareName": "SimpleFlight",
    "DefaultVehicleState": "Armed"
  }
}
