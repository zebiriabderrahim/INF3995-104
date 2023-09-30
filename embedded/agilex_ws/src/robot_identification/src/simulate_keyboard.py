from evdev import UInput, ecodes as e
import os

# Check if the script is running as root
if os.geteuid() != 0:
    print("This script needs to run as root to access /dev/uinput")
    exit(1)

# Create a UInput instance
ui = UInput()

# Simulate typing a hyphen (-)
ui.write(e.EV_KEY, e.KEY_MINUS, 1)
ui.write(e.EV_KEY, e.KEY_MINUS, 0)
ui.syn()

# Close the UInput instance
ui.close()
