def init_glut(self,window_size = (800,600)):
  self.glutwindow = GLUTWindow(self.world, "FWMAV")
  GLUT.glutInit(())
  GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA |
         GLUT.GLUT_DOUBLE |
         GLUT.GLUT_MULTISAMPLE |
         GLUT.GLUT_ALPHA |
         GLUT.GLUT_DEPTH)
  GLUT.glutInitWindowSize(*window_size)
  GLUT.glutInitWindowPosition(0, 0)
  self.glutwindow.window = GLUT.glutCreateWindow(self.glutwindow.title)
  self.glutwindow.initGL(*window_size)
  self.camera_theta = 75
  self.camera_phi = 90
  self.camera_horizontal = 0.0
  self.camera_vertical = -0.25
  self.camera_depth = -1.5
  self.camera_angle_increment = 5
  self.camera_position_increment = 0.05
  self.update_camera()
  #self.glutwindow.scene.add_camera(Trackball(theta = self.camera_theta, phi = self.camera_phi, trans=[self.camera_horizontal, self.camera_vertical, self.camera_depth]),"Camera Z up close")
  #self.glutwindow.scene.set_camera(2)
  self.glutwindow.scene.resize(*window_size)
  self.glutwindow.drawGL()
# Flappy

Flappy Hummingbird: An Open Source Dynamic Simulation of Flapping Wing Robots and Animals

This work has been published and presented at ICRA2019 (The 2019 International Conference on Robotics and Automation). 

It will be updated continuously. Stay tuned.

![](demo.gif)

## Publication

To cite this work in publications:

	@inproceedings{fei2019flappy,
	  title={Flappy Hummingbird: An Open Source Dynamic Simulation of Flapping Wing Robots and Animals},
	  author={Fei, Fan and Tu, Zhan and Yang, Yilun and Zhang, Jian and Deng, Xinyan},
	  booktitle={2019 IEEE International Conference on Robotics and Automation (ICRA)},
	  pages={9223--9229},
	  year={2019},
	  organization={IEEE}
	}

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Flappy requires python3 with the development headers. You'll also need some other system packages. DART is required as a simulation engine (and we use pydart2 as interface). They can be installed as follows

#### Ubuntu

	# install system packages
	sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev swig python3-pip python3-pyqt4 python3-	pyqt4.qtopengl
	pip install click
	
### Install DART from source
[Install DART from soucrce](https://dartsim.github.io/install_dart_on_ubuntu.html)
* Please install all the dependencies so there will less likely to have errors when installing the repository
* Make sure you install version 6.2.1 by changing 
	
	```zsh
	git checkout tags/v6.8.2
	```
	to	
	```zsh
	git checkout tags/v6.2.1
	```
Please refer the [official DART installation document](https://github.com/dartsim/dart/wiki/Installation) when you have problems. 

#### Mac OS X(not finished)
Installation of system packages on Mac requires [Homebrew](https://brew.sh). With Homebrew installed, run the follwing:
```bash
# install system packages
brew install cmake openmpi

# install dart
brew install dartsim
```
### Alternative installation methods
1. Pydart2 source code included in our project. (No extra installation required) 
	[No installation](https://github.com/chen3082/flappy)
2. Pydart2 source code not included in our project. (Installation required, which is not a pleasant experience.) 
	* Please continue your steps here
3. Docker (If you do not have access to recommanded OS environment) 
	[Docker container](https://github.com/chen3082/docker)

### Virtual environment
From the general python package sanity perspective, it is a good idea to use virtual environments (virtualenvs) to make sure packages from different projects do not interfere with each other. Creation of virtual environments is done by executing the command [venv](https://docs.python.org/3/library/venv.html#module-venv):

```zsh
python3 -m venv /path/to/new/virtual/environment
```

To activate a venv:

```zsh
source /path/to/venv/bin/activate
```

### Installation
- Pydart2 is a python binding to DART. 

    ```zsh
    pip install pydart2
    ```
    If this does not work please try to install from source.
    Please refer to [document](https://pydart2.readthedocs.io/en/latest/install.html) when you have problems.

- [Tensorflow](https://github.com/tensorflow/tensorflow) is needed for the usage of neural network. If you want to make use of your GPU, please install the tensorflow with gpu
- Please make sure you install tensorflow version < 2.0

    ```zsh
    pip install tensorflow-gpu # if you have a CUDA-compatible gpu and proper drivers
    ```
    else
    ```zsh
    pip install tensorflow==1.9
    ```
    please refer to [TensorFlow installation guide](https://www.tensorflow.org/install/)
    for more details. 
  
- Clone the repo and cd into it:
    ```zsh
    git clone https://github.com/purdue-biorobotics/flappy.git
    cd flappy
    ```
    
- Install Flappy package
    ```zsh
    pip install -e .
    ```

- Lack dependency
   * Please try to install any dependency if there is an error related to that library during the installation process.
   ```zhs
   pip [Dependency lack of]
   or
   apt-get install [Dependency lack of]
   ```
   
## Environments
### FWMAV
Dual motor driven flapping wing robots based on the Purdue Hummingbird robot.

The control of this vehicle is a difficult problem. We challenge developers, researchers, scientists and roboticists to come up with better control algorithms, either feedback controller or learning-based controller.

Two default working controller are included: a cascading PID controller (control structure similar to ArduPilot) and an Adaptive Robust Controller (ARC). These two controllers can be evaluated in the provided test script.

#### 'fwmav_hover-v0'
This environment is for controlling the dual wing flappin wing robot.

The four inputs are the thrust and torque signals:
    * voltage_amplitude_ [0,18] volt for thrust
    * differential_voltage_ [-3,3] volt for roll torque
    * mean_voltage_ [-3.5,3.5] volt for pitch torque
    * split_cycle_ [-0.15,0.15] for yaw torque

This mode of control is similar to helicopter or quadcopter control. The sinusoidal voltage signal drives the wing back and forth is generated by using [wing beat modulation](https://arc.aiaa.org/doi/10.2514/1.47146).

The input actions are in [-1,1] and will be scaled to their appropriate range. If implementing a feedback controller, the input should be scaled to [-1,1]. See the baseline PID controller and test example for detail.

##### Testing with Closed loop controller (PID or ARC)
```zsh
python test.py --model_type=PID
python test.py --model_type=ARC
```

##### Learning
We choose to use stable baselines instead of baselines as our RL library. Note that our environment still follows the specification of the gym/env, so baselines can be applied to our env as well.

###### Training
```zsh
python train.py --model_type=PPO2 --model_path=ppo2_mlp --policy_type=MlpPolicy --n_cpu=12 --time_step=100000
```

###### Testing with trained model
```zsh
python test.py --model_type=PPO2 --model_path=ppo2_mlp --policy_type=MlpPolicy
```

#### 'fwmav_hover-v1'
This environment similar to `'fwmav_hover-v0'`, but without using [wing beat modulation][doman2010wingbeat].

An example using PID is in `test_simple.py`, this example still uses the same PID controller and wing beat modulation. But the input to the environment are just two voltage signals ([-18,18] volt) supplied to the two motors.

To develop a new control policy, the policy should generate sinusoidal signals near 34Hz to drive the wings and implement control at the same time. Without specifying the wing kinematics allows the ability to generate torque and force in arbitrary directions.

##### Testing with Closed loop controller (PID or ARC)
```zsh
python test_simple.py --model_type=PID
python test_simple.py --model_type=ARC
```


## Contributor
Fan Fei, Ruoyu Wu, Jian Zhang, Zhan Tu, Yunlei Yan, Yuan-Cheng Chen

## License
MIT

## Acknowledments

