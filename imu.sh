#!/bin/sh
### BEGIN INIT INFO
# Provides:          imutest
# Required-Start:    $local_fs $syslog $remote_fs
# Required-Stop:     $local_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: IMU, Camera data
# Description:       Commands can collects both video form camera & burst data from IMU
### END INIT INFO

IMU_BIN=/usr/bin/imutest
test -x $IMU_BIN || { echo "$IMU_BIN not installed";
	if [ "$1" = "stop" ]; then exit 0;
	else exit 5; fi; }



case "$1" in
	start)
		echo "Starting IMU test"	
		$IMU_BIN &> /var/log/imutest.log
		;;
	stop)
		echo "Stopping IMU test"
	
		killall imutest
		;;
	*)
	    echo "Usage: /etc/init.d/imu.sh {start|stop}"
	    exit 1
	    ;;
esac

exit 0
