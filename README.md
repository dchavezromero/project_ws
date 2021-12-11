# RBE 501 Project
Final project for RBE501

## IMPORTANT!
This project was developed for Ubuntu 18.04. and ROS Melodic.

## Dependencies
### `catkin_tool` for setting the build space of our project
```
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
```
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### `rospkg` for Python2
```
pip install rospkg
```

### OpenCV for Python2
Needed for computer vision-related packages that the `apriltag_ros` depends on
```
pip2 install opencv-python==4.2.0.32
```

### `sympy` for Python2
```
pip2 install sympy
```

### `modern_robotics` for Python2
```
pip install modern_robotics
```

## Installation instructions
```
cd ~
```
To clone the project (needs SSH key setup on your machine -- URL for tutorial: https://docs.github.com/en/enterprise-server@3.0/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
```
git clone --recursive git@github.com:dchavezromero/project_ws.git
```
Build the workspace
```
cd ~/project_ws
```
```
catkin build
```
Finally, source your `~/.bashrc` and the project folder `setup.bash` files
```
source ~/.bashrc && source ~/project_ws/devel/setup.bash
```


### To calibrate your camera and print a sample checkerboard for your calibration
Useful links:

https://www.youtube.com/watch?v=UGArg1kQwFc&list=PLI79e6UyigXUgY9uMvNvmbnWqMfh0j2Oh&index=5

https://markhedleyjones.com/projects/calibration-checkerboard-collection

(NOTE: when running the `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.023 image:=/usb_cam/image_raw camera:=/usb_cam` command, you must specify the `--size 8x6` flag to match the dimensions of your checkerboard -1.
For example, for a checkerboard of size 11x8 square you would input `--size 10x7`)

Remember to update your `~/project_ws/src/apriltag_ros/apriltag_ros/scripts/camera_info.py` node with your custom parameters from the camera calibration :)

### IMPORTANT

We need to prevent git from tracking the `camera_info.py` node that keeps each one of our individual camera parameters.

To do this

```
cd ~/project_ws/src/apriltag_ros
```
```
git update-index --assume-unchanged ~/project_ws/src/apriltag_ros/apriltag_ros/scripts/camera_info.py
```
Once you are done calibrating run
```
catkin build
```

### Print a sample apriltag from the tag36h11 family from this link

https://mega.nz/file/BJY3yQyQ#xkId2YbRF0LyPksN9ykok3dREEPiXEPVEMFOuhKWmt4

## Running the sample apriltag detection node
```
cd ~/project_ws && source ~/project_ws/devel/setup.bash
```
On a different set of terminals
```
roslaunch apriltag_ros continuous_detection.launch
```
```
rqt_image_view
```
```
roslaunch panda_arm panda_arm_rviz.launch
```
Place the arpiltag in front of your camera.
On RVIZ, change the base frame to `tagN` and then to `world` 
