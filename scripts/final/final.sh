#!/bin/bash
python takeoff.py
rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"
python3 final_integration.py