#!/bin/bash
source /home/cgz/catkin_ws/devel/setup.bash

# Cleanup
pkill -f ros
pkill -f gazebo
pkill -f fgo_node
sleep 2

LOG_DIR=/home/cgz/catkin_ws/src/rexrov_fgo_nav/logs
mkdir -p $LOG_DIR

echo "Starting Simulation..."
roslaunch rexrov_fgo_nav start_simulation.launch > $LOG_DIR/sim.log 2>&1 &
PID_SIM=$!
sleep 15

echo "Starting Controller..."
roslaunch uuv_control_utils start_circular_trajectory.launch uuv_name:=rexrov radius:=10 center_z:=-20 > $LOG_DIR/control.log 2>&1 &
PID_CTRL=$!
sleep 5

echo "Starting FGO..."
roslaunch rexrov_fgo_nav start_fgo_nav.launch > $LOG_DIR/fgo.log 2>&1 &
PID_FGO=$!

echo "Waiting for data (30s)..."
sleep 30

echo "--- FGO LOG TAIL ---"
tail -n 40 $LOG_DIR/fgo.log

echo "Killing processes..."
kill $PID_FGO $PID_CTRL $PID_SIM
pkill -f ros
