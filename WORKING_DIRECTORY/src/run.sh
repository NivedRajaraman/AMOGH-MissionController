#!/bin/bash;
gnome-terminal --command="./main" &
wait
gnome-terminal --command="./shm" &
wait
gnome-terminal --command="python ./SIMULATION/Simulator.py"
wait
gnome-terminal --command="python ./SIMULATION/Environment.py"

