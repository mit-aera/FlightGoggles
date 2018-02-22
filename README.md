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

## Citation
If you find this work useful for your research, please cite:
```bibtex
@article{mccord2018quadcopter,
  
  title={Visual-inertial navigation algorithm development
  using photorealistic camera simulation in the loop},
  
  author={Thomas Sayre-McCord, Winter Guerra, Amado Antonini,
  Jasper Arneberg, Austin Brown, Guilherme Cavalheiro, Yajun Fang,
  Alex Gorodetsky, Dave McCoy, Sebastian Quilter, Fabian Riether,
  Ezra Tal, Yunus Terzioglu, Luca Carlone, and Sertac Karaman},
  
  journal={ICRA 2018},
  year={2018}
}
```
