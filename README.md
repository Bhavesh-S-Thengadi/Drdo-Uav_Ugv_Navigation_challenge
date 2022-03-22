## DRDO's UAV-Guided UGV Navigation Challenge

Python -version => 3.6.9

### Library Used: 
- rospy => 1.14.12
- numpy => 1.19.5
- pencv-python => 4.5.5.64
- cv_bridge => 1.13.0
- imutils => 0.5.4
- mavros => 1.13.0
- math
- abc
- imp

# Imports for ROS Publishers and Subscribers:
- sensor_msg.msg –Image
- td_msgs.msg –String
- geometry_msgs.msg –Twist
- prius_msgs.msg –Control

# Instructions for running step by step
1.Launching the world and respective consoles:
- Run the <file_name.sh> for launching of the respective world and starting the ardupilot/Arducopter, sim_vehicle.py and gazebo-iris –console.
       
       `./<path><file_name>.sh`

2.Wait for the GPS:
- Note: Prearm checks should be disabled for avoiding the Gyros not calibrating problem because of the car slipping on the road during the start.
- Automatic Takeoff:
1.Run the <file_name.sh> that will launch the Takeoff script of the UAV till <height>m.
2.This file would also call a ros service that would change the global frame of the drone to its local_ned frame “8”.

       ./<path><file_name>.sh

3.This file would also run the main <final_intigration>.py which has the code for running the world. This .py file would also import other required .py files those are provided in the package.
