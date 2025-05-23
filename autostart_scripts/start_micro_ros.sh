#!/bin/bash

rosws=dev_ws
rospackage=zmoab_uros_utils

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/micro_ros.log

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



while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting micro_ros_agent" >> $LOGFILE
		
		ros2 run micro_ros_agent micro_ros_agent  serial --dev /dev/ttyACM0 >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
