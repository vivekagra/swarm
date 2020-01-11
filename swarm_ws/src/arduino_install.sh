#!/bin/bash

sudo apt-get update
sudo apt-get install arduino arduino-core

# install ros serial
sudo apt-get update
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial

## use your user name here
# sudo usermod -a -G dialout YOUR_USER
