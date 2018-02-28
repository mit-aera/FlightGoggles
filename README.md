# FlightGoggles
A lightweight framework for hardware-in-the-loop agile flight simulation using Unity and LCM.

[![Video Link](https://img.youtube.com/vi/_VBww8YQuA8/0.jpg)](https://www.youtube.com/watch?v=_VBww8YQuA8)

## Getting Started

### Requirements

* Unity Editor version >= `2017.3.1` running on Windows. You can find the latest releases of Unity [here](https://unity3d.com/get-unity/download/archive).
* Unity must have experimental .NET APIs enabled in the graphics project:
	+ `Edit -> Project Settings -> Player -> Scripting Runtime Version = 4.6`


Clone this repo and some required repos into your Unity Project's `Assets` folder using the following commands:

```bash
cd <UNITY_PROJECT_DIRECTORY>
# Clone repos & submodules
git clone --recursive https://github.com/AgileDrones/FlightGoggles.git
```

### Getting Client Bindings

Download the [client bindings](https://github.com/AgileDrones/FlightGogglesClientBindings) from the linked repo using the instructions in that repo.

## Citation
If you find this work useful for your research, please cite:
```bibtex
@inproceedings{sayremccord2018visual,
  title={Visual-inertial navigation algorithm development using photorealistic camera simulation in the loop},
  author={Sayre-McCord, Thomas and
  Guerra, Winter and
  Antonini, Amado and
  Arneberg, Jasper and
  Brown, Austin and
  Cavalheiro, Guilherme and
  Fang, Yajun and
  Gorodetsky, Alex and
  McCoy, Dave and
  Quilter, Sebastian and
  Riether, Fabian and
  Tal, Ezra and
  Terzioglu, Yunus and
  Carlone, Luca and
  Karaman, Sertac},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2018}
}
```
