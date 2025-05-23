#!/bin/bash

rosws=dev_ws
rospackage=zmoab_uros_utils

sleep 30

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/amcl.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt

FILESIZE=$(stat -c%s "$LOGFILE")
FILESIZE_LIMIT=200000000
if [ "$FILESIZE" -gt "$FILESIZE_LIMIT" ];
then
    echo "LOGFILE is $FILESIZE, remove the file"
	rm $LOGFILE
else
    echo "LOGFILE is $FILESIZE" >> $LOGFILE
fi

params_file=/home/raspberry/dev_ws/src/zmoab_uros_utils/config/nav2_amcl.yaml
map_file=/home/raspberry/map/office.yaml

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting localization_launch_humble" >> $LOGFILE
		
		ros2 launch  zmoab_uros_utils localization_launch_humble.py params_file:=$params_file map:=$map_file >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
