#!/bin/bash;


gnome-terminal --command="g++ -std=c++11 MissionController.cpp main.cpp state.cpp BasicController.cpp LaneController.cpp BuoyController.cpp MarkerController.cpp GateController.cpp -o main" &
sleep 5 &
wait
gnome-terminal --command="./main" &
gnome-terminal --command="g++ -std=c++11 ./PYTHON_INTERFACE/SharePWM.cpp state.cpp -o shm" &
sleep 5 &
wait
gnome-terminal --command="./shm" &
sleep 1 &
wait
gnome-terminal --command="python ./PYTHON_INTERFACE/Simulator.py"

