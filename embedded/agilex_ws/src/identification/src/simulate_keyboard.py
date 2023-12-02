from evdev import UInput, ecodes as e
import os

if os.geteuid() != 0:
    print("This script needs to run as root to access /dev/uinput")
    exit(1)

ui = UInput()

ui.write(e.EV_KEY, e.KEY_MINUS, 1)
ui.write(e.EV_KEY, e.KEY_MINUS, 0)
ui.syn()

ui.close()
