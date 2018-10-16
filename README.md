# asarob_demo

The following instructions provide some basic information to the usage of Care-O-bot and the woz use case developed for T-Labs.

## Basic Robot Usage
Instructions for an introduction and the basic usage of the Care-O-bot 4 robot can be found here:
- http://wiki.ros.org/Robots/cob4/manual
- http://wiki.ros.org/Robots/Care-O-bot

Please read the user manual carefully before working with the robot. Give yourself some time to try out the functions outlined in the user manual to get acquinted with the robot. Whenever you command movements, especially arm or torso movements, keep the remote emergency button in your hands, watch the robot carefully and be prepared to stop the robot whenever something unexpected happens.

## Logging in to the robot
Connect to the WiFi network *'cob4-7-direct'*. WiFi-Password is *'care-o-bot'*. <br>
The basic user for working and developing with the robot is *'asarob'*. The password is *'asarob'*.

## Unloading from Box and Stop Scenario


1. Log in to the robot base PC, use the standard drivers to unload from box:
   ```
   ssh -X telekom@b1.cob4-7
   rosservice call /docker_control/undock station_travel_box
   ```


2. Log in to the robot base PC, use the standard drivers to stop state machine scenario:
   ```
   ssh -X telekom@b1.cob4-7
   rosservice call /behavior/stop_state_machine
   ```   

## Running Scenario

1. Log in to the robot base PC, stop the standard drivers (if running) and load the telekom-specific robot drivers:
   ```
   ssh -X telekom@b1.cob4-7
   sudo cob-stop
   sudo cob-command stop_core
   roslaunch cob_bringup robot.launch
   or 
   roslauch asarob_demo ipa_brinup.launch
   ```
   than initialize all driver via joysticks
   
2. Log in to the robot head PC and start the face camera:
   ```
   ssh -X telekom@h1.cob4-7
   roslaunch asarob_demo cam3d_realsense2_rgbd.launch
   ```
3. Export the ROS master URI to the robot and start RViz on your external computer:
   ```
   export ROS_MASTER_URI=http://b1.cob4-7:11311
   ```
There the following the list of additional software repositories is need for running the demonstration use case:
- cob_robots (```git clone -b woz_karlsruhe https://github.com/ipa-mjp/cob_robots.git```)
- cob_calibration_data (```git clone -b woz_leipzig https://github.com/ipa-mjp/cob_calibration_data.git```)
