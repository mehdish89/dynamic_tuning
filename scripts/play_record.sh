#!/bin/bash

path="/home/osrf/catkin_ws/src/dyn_tune/scripts"
sname="$path/$1"
echo $sname

if [[ $2 = *[!\ ]* ]]; then
  echo "\$2 contains characters other than space"
  rosbag play -d 1 -p "$2" $sname &
  dname="$path/exp-sim-$1"
else
  echo "\$2 consists of spaces only"
  rosbag play -d 1 $sname &
  dname="$path/exp-real-$1"
fi

# echo "$2/follow_joint_trajectory/.*|/joint_states"
rosbag record --duration=15 -O $dname -e "$2/follow_joint_trajectory/.*|/joint_states"

