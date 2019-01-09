#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

set -eu
stdbuf -o L roslaunch neonavigation_launch demo.launch &
pid=$!

while true
do
  if rostopic echo -n1 /planner_3d/status | grep "status: 0"
  then
    break
  fi
  sleep 1
done

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: \"map\"
pose: 
  position: 
    x: 3.3
    y: 6.2
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.707106781187
    w: 0.707106781187" &

while true
do
  if rostopic echo -n1 /planner_3d/status | grep "status: 1"
  then
    break
  fi
  sleep 1
done

rosrun tf tf_echo map base_link 1 | grep --line-buffered Translation | tee /tmp/pos.dat &

error=false
cnt=0
while true
do
  if rostopic echo -n1 /planner_3d/status | grep "status: 0"
  then
    echo "Reached to the goal"
    break
  fi
  sleep 1
  cnt=`expr $cnt + 1`
  if [ $cnt -gt 60 ]
  then
    error=true
    break
  fi
done

kill -SIGINT $pid
wait $pid

echo "========="
if [ $error == "true" ]
then
  echo " ERROR"
  exit 1
else
  position=`tail -n1 /tmp/pos.dat | sed -e "s/^.*\[//;s/\].*$//;s/,//g"`
  x=`echo $position | cut -f1 -d" "`
  y=`echo $position | cut -f2 -d" "`
  pos_error=`echo "sqrt(($x - 3.3)^2 + ($y - 6.2)^2)" | bc`
  echo " - position error is $pos_error"
  if [ `echo "$pos_error < 0.5" | bc` == "1" ]
  then
    echo " SUCCESS"
  else
    echo " ERROR"
    exit 1
  fi
fi
