#!/bin/bash;
gnome-terminal --command="g++ -std=c++11 MissionController.cpp main.cpp state.cpp BasicController.cpp LaneController.cpp BuoyController.cpp MarkerController.cpp GateController.cpp -o main" &

gnome-terminal --command="g++ -std=c++11 ./PYTHON_INTERFACE/SharePWM.cpp state.cpp -o shm"
