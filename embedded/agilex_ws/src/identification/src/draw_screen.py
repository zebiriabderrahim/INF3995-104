#!/usr/bin/env python3
import subprocess

command = 'Xvfb :99 -screen 0 1024x768x24 & gnome-terminal --display=:99 --full-screen -- bash -c "for i in {1..3}; do sl; done; exit"'

subprocess.call(command, shell=True)
subprocess.call(['sudo', 'python', '/home/nvidia/agilex_ws/src/src/scripts/identification/src/simulate_keyboard.py'])

