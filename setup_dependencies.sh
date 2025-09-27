#!/bin/bash
set -e

sudo apt update
sudo apt install -y python3-pip ros-humble-tf2-geometry-msgs
pip3 install --user transforms3d transformations
