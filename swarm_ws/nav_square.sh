#!/bin/bash

rosrun nav_behaviors nav_timed.py _time_to_run:=2 _vx_target:=0.01 _vy_target:=0.0 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _vx_target:=0.00 _vy_target:=0.01 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _vx_target:=-0.01 _vy_target:=0.0 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _vx_target:=0.00 _vy_target:=-0.01 _w_target:=0.0
