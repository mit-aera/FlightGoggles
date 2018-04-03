# Editing FlightGoggles

## Requirements

* Unity Editor version >= `2017.4.1` running on Windows. You can find the latest releases of Unity [here](https://unity3d.com/get-unity/download/archive).

## Quickstart

Download the [starter Unity3D project folder](https://github.com/AgileDrones/FlightGoggles/releases/download/v1.3.0/Unity_Project_Folder_Flight_Goggles_Public_v1.3.0.zip).

```bash
# Update the FlightGoggles code inside the starter project.
cd Assets/FlightGoggles
git pull
```

Due to licensing, we cannot provide you with some of the paid software modules that are included in the binary. However, these modules are easy to add back to the starter project.

The modules we use are:
* [TriLib OBJ/DAE/FBX loader](https://assetstore.unity.com/packages/tools/modeling/trilib-unity-model-loader-package-91777)
* [CTAA Cinematic Temporal Anti-Aliasing](https://assetstore.unity.com/packages/vfx/shaders/ctaa-cinematic-temporal-anti-aliasing-pc-vr-106995)

Neither of these packages are neccessary for core FlightGoggles operation. TriLib can be disabled by uncommenting `#define TRILIB_DOES_NOT_EXIST` in [CameraController.cs](./Scripts/CameraController.cs). Additionally, FlightGoggles will fall back to the Unity FXAA antialiaser if it cannot find CTAA.