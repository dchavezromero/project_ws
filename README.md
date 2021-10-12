# RBE 501 Project
Final project for RBE501

## Dependencies
### `catkin_tool` for setting the build space of our project
```
cd ~
sudo apt-get update
udo apt-get install python3-catkin-tools
```

### OpenCV for Python2
Needed for computer vision-related packages that the `apriltag_ros` depends on
```
pip install opencv-python
```

### usb_cam ROS Package
Needed to get the webcam feed and carry out the inital camera calibration
```
cd ~/catkin_ws/src
```
```
git clone git@github.com:ros-drivers/usb_cam.git
```
```
catkin_make && source ~/catkin_ws/devel/setup.bash
```

### apriltag_ros ROS Package
Needed to perform the live, continous detection of aprilags on-screen
```
cd ~/catkin_ws/src
```
```
git clone git@github.com:AprilRobotics/apriltag_ros.git
```
```
catkin_make && source ~/catkin_ws/devel/setup.bash
```

To run the sample apriltag detection node
```
cd catkin_ws
```
On a different terminal:
```
rqt_image_view

```
```
rosrun rviz rviz
```
Add the `/tf` topic, put the apriltag on-screen, and change the base frame to `camera` 
