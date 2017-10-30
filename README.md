# FlightGoggles
A lightweight framework for hardware-in-the-loop agile flight simulation using Unity and LCM.

@TODO: Video Link

## Getting Started

### Requirements

* Unity Editor version >= `2017.3.0b6` running on Windows or Linux. You can download the latest beta version of Unity [here](https://unity3d.com/unity/beta).
* Unity must have experimental .NET APIs enabled in the graphics project:
	+ `Edit -> Project Settings -> Player -> Scripting Runtime Version = 4.6`


Clone this repo and some required repos into your Unity Project's `Assets` folder using the following commands:

```bash
cd <UNITY_PROJECT_DIRECTORY>
# Clone repos
git clone --recursive git@github.com:AgileDrones/FlightGoggles.git
```

## Citation
If you find this work useful for your research, please cite:
```bibtex
@article{mccord2017quadcopter,
  title={A Vision-based Agile Quadcopter Enabled by On-board Embedded High-performance Computing},
  author={R. Thomas Sayre-McCord, Amado Antonini, Jasper Arneberg, Austin Brown, Guilherme Cavalheiro, Yajun Fang, Alex Gorodetsky, Winter Guerra, Dave McCoy, Sebastian Quilter, Fabian Riether, Ezra Tal, Yunus Terzioglu, Luca Carlone, and Sertac Karaman},
  journal={arXiv preprint arXiv:____},
  year={2017}
}
```
