#!/bin/bash
# file: startup.sh
echo -e "\e[36m======================================================================"
echo "DO NOT RUN THIS SCRIPT AS SUPERUSER!"
echo "Starting ros services on Willy in separate screens. To monitor output:"
echo "use 'screen -r [SCREENNAME]' to attach to a screen."
echo "use 'screen -ls' to list all active screens."
echo "======================================================================"
echo "Starting roscore in screen named 'roscore'..."
screen -dmS roscore ./roscore.sh
sleep 2
echo "Starting Arduino's on ttyACM0 and ttyACM1 in screen named 'Arduino0_1'..."
screen -dmS Arduino0_1 ./launchTTYACM0_1.sh
sleep 2
echo "Starting Arduino on ttyACM2 in screen named 'Arduino2'..."
screen -dmS Arduino2 ./launchTTYACM2.sh
sleep 2
echo -e "======================================================================"
echo "Roscore and all nodes(arduino's) connected with Willy have been started."
echo -e "\e[0m"

