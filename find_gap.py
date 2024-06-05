"""Finds the geometry gap"""

import json
import threading
from time import sleep, time
from datetime import datetime
import re
from Actuator.ticactuator import TicActuator
from LoadCell.openscale import OpenScale

# - Initialization -------------------------------------------

date = datetime.now()
date_str = date.strftime("%Y-%m-%d_%H-%M-%S")
"""timestamp string for experiment start time in yyyy-mm-dd_HH:MM:SS format"""
CSV_NAME = date_str + "_" + "find_gap" + "-data.csv"
CONFIG_PATH = "LoadCell\\config.json"

N_FIND = 25

force: float = 0
"""Current force reading. Positive is a force pushing up on the load cell"""
FORCE_UP_SIGN = -1
"""Sign of a positive force. This should be 1 or -1, and is used to compute
velocity based on force"""
start_gap: float = 0
"""Initial distance away from hard stop."""
gap: float = 0
"""Current gap between hammer and hard stop"""

if __name__ == "__main__":

    scale = OpenScale()

    # target_line = input("Enter the target force in [{:}]: ".format(scale.units))
    # temp = re.compile("[0-9.]+")
    # res = temp.search(target_line).group(0)
    # target = float(res)
    # print("Target force is {:.2f}{:}".format(target, scale.units))

    gap_line = input("How far should I go out to start, in mm? ")
    temp = re.compile("[0-9.]+")
    res = temp.search(gap_line).group(0)
    start_gap = float(res)
    print(f"Starting gap is {start_gap:.2f}mm")

    actuator = TicActuator(step_mode=5)
    actuator.set_max_accel_mmss(20, True)
    actuator.set_max_speed_mms(5)

    # Zero current motor position
    actuator.halt_and_set_position(0)
    actuator.heartbeat()

    with open("data/" + CSV_NAME, "a") as datafile:
        datafile.write(
            (
                "Current Time, Elapsed Time, Current Position (mm), Current Position,"
                "Target Position, Current Velocity (mm/s), Current Velocity, Target Velocity,"
                "Max Speed, Max Decel, Max Accel, Step Mode, Voltage In (mV), Current Force"
                f"({scale.units}) Current Gap (m)\n"
            )
        )


def load_cell_thread():
    """Continuously reads load cell and reports the upward force on the load cell"""
    global force

    start_time = time()

    for _ in range(10):  # get rid of first few lines that aren't readings
        scale.get_line()
    scale.flush_old_lines()  # and get rid of any others that were generated
    # when we were busy setting up
    while True:
        force = scale.wait_for_calibrated_measurement(True) * FORCE_UP_SIGN

        if (time() - start_time) >= 2000 or (
            (not ac.is_alive()) and (not b.is_alive()) and (time() - start_time) > 1
        ):
            print("Stopping load cell reading")
            return


def actuator_thread():
    """Drives actuator"""
    global gap

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    actuator.startup()

    approach_velocity = -0.5  # mm/s, speed to approach hard stop at
    force_threshold = 0.8  # g, the force to look for when you hit something
    backoff_dist = 0.1  # mm, distance to back away when you hit

    hit_pos = 0

    # Go to starting gap
    actuator.move_to_mm(-abs(start_gap))
    print("Reached start point. Now approaching to find the hard stop slowly.")

    # Start by approaching and waiting until force is non-negligible
    actuator.set_vel_mms(approach_velocity)
    while True:
        # print("{:6.2f} <? {:6.2f}".format(force, force_threshold))
        actuator.heartbeat()
        if abs(force) > scale.force_limit:
            print("Force was too large, stopping.")
            actuator.go_home_quiet_down()
            return
        if abs(force) > force_threshold:
            hit_pos = actuator.get_pos_mm()
            print(f"Hit something at {hit_pos:.2f}mm")
            actuator.move_to_mm(hit_pos + backoff_dist)
            break
        # print("{:6.2f} >=? {:6.2f}".format(get_pos_mm() / 1000.0, start_gap))
    print("Force threshold met, switching over to fine approach.")

    gap_list = [0] * N_FIND
    slowdown_factor = 0.05  # what factor to slow down by on fine approach

    for i in range(N_FIND):
        actuator.set_vel_mms(approach_velocity * slowdown_factor)
        while True:
            # Check if force beyond max amount
            if abs(force) > scale.force_limit:
                print("Force was too large, stopping.")
                actuator.go_home_quiet_down()
                return

            # Check if we reached something with a lighter force
            if abs(force) > force_threshold:
                hit_pos = actuator.get_pos_mm()
                gap_list[i] = hit_pos
                if i < N_FIND - 1:
                    print(
                        f"Hit something at {hit_pos:.4f}mm, backing up to check again"
                    )
                else:
                    print(f"Hit something at {hit_pos:.4f}mm, done checking")
                actuator.move_to_mm(hit_pos + backoff_dist)
                break

            gap = actuator.get_pos_mm()
            out_str = f"{(i + 1):2d}: {force:7.3f}{scale.units}, pos = {gap:8.3f}mm"

            print(out_str)
            # print(actuator.variables.error_status)
            actuator.heartbeat()

    mean_gap = abs(sum(gap_list) / N_FIND)
    print(f"The mean gap is {mean_gap:f}mm")

    # Save gap in config file
    try:
        with open(CONFIG_PATH, "r") as read_file:
            config = json.load(read_file)
    except:
        config = {}
    config["gap"] = mean_gap
    with open(CONFIG_PATH, "w") as write_file:
        json.dump(config, write_file)

    actuator.go_home_quiet_down()
    print("Done with actuator")


def background():
    """Records data to csv"""

    start_time = time()
    while True:
        cur_pos_mm = actuator.get_pos_mm()
        cur_pos = actuator.get_pos()
        tar_pos = actuator.variables.target_position
        cur_vel_mms = actuator.get_vel_mms()
        cur_vel = actuator.get_vel()
        tar_vel = actuator.variables.target_velocity
        max_speed = actuator.variables.max_speed
        max_decel = actuator.variables.max_decel
        max_accel = actuator.variables.max_accel
        step_mode = actuator.variables.step_mode
        vin_voltage = actuator.variables.vin_voltage

        with open(
            "data/" + CSV_NAME, "a"
        ) as datafile:  # write time & current details to csv
            cur_time = time()
            cur_duration = cur_time - start_time

            output_params = [
                cur_time,
                cur_duration,
                cur_pos_mm,
                cur_pos,
                tar_pos,
                cur_vel_mms,
                cur_vel,
                tar_vel,
                max_speed,
                max_decel,
                max_accel,
                step_mode,
                vin_voltage,
                force,
                gap,
            ]
            dataline = ",".join(map(str, output_params)) + "\n"
            datafile.write(dataline)
            # print(dataline)

        sleep(0.1)

        if (time() - start_time) >= 2000 or (
            (not ac.is_alive()) and (time() - start_time) > 1
        ):
            print("end of print")
            break

    print("=" * 20 + " BACKGROUND IS DONE " + "=" * 20)


lc = threading.Thread(name="loadcell", target=load_cell_thread)
ac = threading.Thread(name="actuator", target=actuator_thread)
b = threading.Thread(name="background", target=background)

lc.start()
ac.start()
b.start()
