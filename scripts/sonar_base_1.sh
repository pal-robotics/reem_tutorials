#!/bin/bash

FREQUENCY=$1

rostopic echo --filter "int(m.header.frame_id[-7:-5]) < 7" /sonar_base | rostopic pub -r $FREQUENCY /sonar_base_1 sensor_msgs/Range >/dev/null

