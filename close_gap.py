import threading
from time import sleep, time
import json
import math
from datetime import datetime
import re
from LoadCell.openscale import OpenScale
from Actuator.ticactuator import TicActuator
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from squeezeflowrheometer import SqueezeFlowRheometer as sfr
import numpy as np

# - Initialization -------------------------------------------

start_gap = 0
"""Initial distance (mm) away from hard stop."""
gap = 0
"""Current gap (m) between hammer and hard stop"""

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    scale = OpenScale()

    # Input test values from external settings file
    settings_path = "test_settings.json"
    with open(settings_path, "r") as read_file:
        settings = json.load(read_file)

    # Get test details from user
    start_gap = sfr.input_start_gap(scale)

    # # Get test details from settings file & config file
    # sample_str = settings["sample_str"]
    # start_gap = float(scale.config["gap"])

    actuator = TicActuator(step_mode=settings["actuator_step_mode"])
    actuator.set_max_accel_mmss(settings["actuator_max_accel_mmss"], True)
    # actuator.set_max_speed_mms(settings["actuator_max_speed_mms"])
    actuator.set_max_speed_mms(5)

    # Zero current motor position
    actuator.halt_and_set_position(0)
    actuator.heartbeat()


def actuator_thread():
    """Drives actuator"""

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    actuator.startup()

    # Move from start position to target gap
    print("Moving to zero gap.")
    target_pos = -start_gap

    # Stop a little bit away
    first_stop = min(0, target_pos + 0.5)
    actuator.heartbeat()
    actuator.move_to_mm(first_stop)
    actuator.heartbeat()

    # Go to final point slowly
    actuator.set_max_speed_mms(0.1)
    actuator.heartbeat()
    actuator.move_to_mm(target_pos)
    actuator.heartbeat()
    print("Reached position, done.")
    print("Actuator thread is done.")


ac = threading.Thread(name="actuator", target=actuator_thread)

ac.start()
