# DetectionOfArucoMarkers

![ROS](https://img.shields.io/badge/ROS-Kinetic_Kame-red.svg)
![MoveIt!](https://img.shields.io/badge/MoveIt-Kinetic_Kame-blue.svg)
![Python](https://img.shields.io/badge/Python-2.7-green.svg)


* subscribes to the topic "/markersAruco" published by tuw_aruco_markers which publishes poses of detected aruco markers 
* publishes averaged poses of detected markers - only for markers, which were detected at least 20x within last 2 seconds

## Install

# Install requirements

1. install RealSense camera drivers and ROS realsense2_camera package according to:
	http://wiki.ros.org/RealSense

2. install tuw_uvc package:
	https://github.com/tuw-robotics/tuw_uvc

3. install tuw_marker_detection package:
	http://wiki.ros.org/tuw_marker_detection?distro=kinetic
(if errors, then    `sudo apt-get install libsdl2-dev`, `sudo apt-get install libsdl1.2-dev`)

4. install marker_msgs package:
	https://github.com/tuw-robotics/marker_msgs

5. add launch files demo_aruco_markermap_realsense.launch and demo_aruco_markermap_webcam.launch (in scripts folder) to tuw_marker_detection/tuw_marker_pose_estimation/launch

6. replace file ArUco.cfg in tuw_marker_detection/tuw_aruco/cfg with the file with the same name in this repository (in scripts folder)

7. Install this package in your workspace, catkin build
8. If does not work, install dependencies, go to your src folder of workspace `rosdep install --from-paths . --ignore-src -r -y`

## Running

* One terminal: 
    1. source current workspace: `source ~/catkin_ws/devel/setup.bash `
    2. `roslaunch realsense2_camera rs_camera.launch` 
* Second Terminal:
    1. source current workspace: `source ~/catkin_ws/devel/setup.bash `
    2. run marker detection: `roslaunch tuw_marker_pose_estimation demo_aruco_markermap_realsense.launch` 
* Third terminal:
    1. source current workspace: `source ~/catkin_ws/devel/setup.bash `
    2. roslaunch static transform and averaged markers position publisher:`roslaunch object_detection_aruco detect_aruco_pose.launch `
* We can see published averaged poses under the topic: `rostopic echo averaged_markers`