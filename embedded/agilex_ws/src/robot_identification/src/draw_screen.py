#!/usr/bin/env python3
import subprocess

# Replace "sl" with your desired command
command = 'Xvfb :99 -screen 0 1024x768x24 & gnome-terminal --display=:99 --full-screen -- bash -c "for i in {1..3}; do sl; done; exit"'

# Execute both the command and the Python script with sudo
subprocess.call(command, shell=True)
subprocess.call(['sudo', 'python', '/home/nvidia/agilex_ws/src/src/scripts/identification/src/simulate_keyboard.py'])

# Alternatively, you can specify the full path to the Python interpreter if needed, e.g.,
# subprocess.call(['sudo', '/usr/bin/python3', 'simulate_keyboard.py'])
