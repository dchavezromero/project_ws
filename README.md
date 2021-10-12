# RBE 501 Project
Final project for RBE501

## Dependencies
### `catkin_tool` for setting the build space of our project
```
cd ~
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### OpenCV for Python2
Needed for computer vision-related packages that the `apriltag_ros` depends on
```
pip install opencv-python
```
### Installation instructions
To clone the project (if you have an SSH key setup on your machine)
```
cd ~
```
```
git clone git@github.com:dchavezromero/project_ws.git
```
Without SSH
```
cd ~
```
```
git clone https://github.com/dchavezromero/project_ws.git
```
Source your `~/.bashrc` and the project folder `setup.bash` files
```
cd ~/project_ws
```
```
source ~/.bashrc && source ~/project_ws/devel/setup.bash
```
### To calibrate your camera and print a sample checkerboard for your calibration
Useful links:
https://www.youtube.com/watch?v=UGArg1kQwFc&list=PLI79e6UyigXUgY9uMvNvmbnWqMfh0j2Oh&index=5
https://markhedleyjones.com/projects/calibration-checkerboard-collection
(NOTE: when running the `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.023 image:=/usb_cam/image_raw camera:=/usb_cam` command, you must specify the `--size 8x6` flag to match the dimensions of your checkerboard - 1.
For example, for a checkerboard of size 11x8 square you would input `--size 10x7`)

Remember to update your `~/project_ws/src/apriltag_ros/apriltag_ros/scripts/camera_info.py` node with your custom parameters from the camera calibration :)

Once you are done calibrating run
```
cd ~/project_ws
```
```
catkin build
```

### To run the sample apriltag detection node
```
cd ~/project_ws && source ~/project_ws/devel/setup.bash
```
```
roslaunch apriltag_ros continuous_detection.launch
```
On a different set of terminals
```
rqt_image_view
```
```
rosrun rviz rviz
```
Add the `/tf` topic, put the apriltag on-screen, and change the base frame to `camera` 
