#!/bin/bash
# file: launchTTYACM2.sh

echo "Starting TTYACM2 as deamon"
rosrun rosserial_python serial_node.py
