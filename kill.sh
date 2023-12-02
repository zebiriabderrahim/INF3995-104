#!/bin/bash

# Set sudo password
sudo_password="nvidia"

# Run lsof to find processes using port 9090
sudo lsof -i :9090

# Kill processes using port 9090
sudo -S <<< $sudo_password lsof -t -i :9090 | xargs -r kill -9

rosnode cleanup purge

rosnode kill -a
