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

date = datetime.now()
date_str = date.strftime("%Y-%m-%d_%H-%M-%S")
"""timestamp string for experiment start time in yyyy-mm-dd_HH:MM:SS format"""
csv_name = date_str + "_" + "set_gap_squeeze_flow_multistep" + "-data.csv"

force = 0
"""Current force reading. Negative is a force pushing up on the load cell"""
target = 0
"""Target force. Negative is a force pushing up on the load cell"""
FORCE_UP_SIGN = 1
"""Sign of a positive force. This should be 1 or -1, and is used to compute velocity based on force"""
start_gap = 0
"""Initial distance (mm) away from hard stop."""
gap = 0
"""Current gap (m) between hammer and hard stop"""
sample_volume = 0
"""Amount of sample (m^3)"""
test_active = False
"""Whether or not the test is active. Should be true after force threshold is met but before test ends"""
sample_str = ""
"""What the sample is made of. Used in filename."""

default_duration = 250
"""Default length of a test in seconds"""
test_duration = 0
"""User-chosen test length in seconds. Selected during input sequence."""

times = []
forces = []
gaps = []

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    scale = OpenScale()

    # Input test values from external settings file
    settings_path = "test_settings.json"
    with open(settings_path, "r") as read_file:
        settings = json.load(read_file)

    # Get test details from user
    start_gap = sfr.input_start_gap(scale)
    test_duration = sfr.input_step_duration(default_duration)
    sample_volume = sfr.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    target_gap_line = input("Enter the target gap in [mm]: ")
    target_gap = sfr.find_num_in_str(target_gap_line)
    """Target gap for polymer stretching"""

    # # Get test details from settings file & config file
    # sample_str = settings["sample_str"]
    # start_gap = float(scale.config["gap"])

    scale.check_tare()

    actuator = TicActuator(step_mode=settings["actuator_step_mode"])
    actuator.set_max_accel_mmss(settings["actuator_max_accel_mmss"], True)
    # actuator.set_max_speed_mms(settings["actuator_max_speed_mms"])
    actuator.set_max_speed_mms(5)

    # Zero current motor position
    actuator.halt_and_set_position(0)
    actuator.heartbeat()

    csv_name = (
        date_str
        + "_"
        + "polymer_stretch_test_{:}_{:d}mL_{:d}mm".format(
            sample_str, round(sample_volume * 1e6), round(target_gap)
        )
        + "-data.csv"
    )

    # Make sure the data folder as well as the figures folder exists before trying to save anything there
    Path("data/Figures/{:}".format(date.strftime("%Y-%m-%d"))).mkdir(
        parents=True, exist_ok=True
    )

    with open("data/" + csv_name, "a") as datafile:
        datafile.write(
            "Current Time,Elapsed Time,Current Position (mm),Current Position,Target Position,Current Velocity (mm/s),Current Velocity,Target Velocity,Max Speed,Max Decel,Max Accel,Step Mode,Voltage In (mV),Current Force ({:}),Start Gap (m),Current Gap (m),Sample Volume (m^3),Test Active?\n".format(
                scale.units
            )
        )


def load_cell_thread():
    """Continuously reads load cell and reports the upward force on the load cell"""
    global force

    start_time = time()

    for _ in range(10):  # get rid of first few lines that aren't readings
        scale.get_line()
    scale.flush_old_lines()  # and get rid of any others that were generated when we were busy setting up

    while True:
        force = scale.wait_for_calibrated_measurement(True) * FORCE_UP_SIGN

        if (time() - start_time) >= 7200 or (
            (not ac.is_alive()) and (not bkg.is_alive()) and (time() - start_time) > 1
        ):
            print("Stopping load cell reading")
            break


def actuator_thread():
    """Drives actuator"""
    global test_active, times, gaps, forces, target, fig

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    actuator.startup()

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 0  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        times = times[-keep_datapoints:]
        forces = forces[-keep_datapoints:]
        gaps = gaps[-keep_datapoints:]

    test_active = True

    # Move from start position to target gap
    print("Target gap is {:.2f}mm".format(target_gap))
    target_pos = target_gap - start_gap
    actuator.heartbeat()
    actuator.move_to_mm(target_pos)
    actuator.heartbeat()
    print("Reached position, waiting")

    rest_start_time = time()
    while time() - rest_start_time <= test_duration:
        sleep(0.1)
        actuator.heartbeat()

    print("Test is done.")
    test_active = False

    # Save fig out before it retracts at end of test
    fig_name = csv_name.replace("-data.csv", "-livePlottedFigure.png")
    fig_path = "data/Figures/{:}/".format(date.strftime("%Y-%m-%d")) + fig_name
    plt.show()
    plt.draw()
    fig.savefig(fig_path, transparent=True)
    print("Actuator thread is done.")


def background():
    """Records data to csv"""
    global actuator, start_gap, test_active

    start_time = time()
    while True:
        cur_pos = actuator.get_pos()
        cur_pos_mm = actuator.steps_to_mm(cur_pos)
        tar_pos = actuator.get_variable_by_name("target_position")
        cur_vel = actuator.get_vel()
        cur_vel_mms = actuator.vel_to_mms(cur_vel)
        tar_vel = actuator.get_variable_by_name("target_velocity")
        max_speed = actuator.get_variable_by_name("max_speed")
        max_decel = actuator.get_variable_by_name("max_decel")
        max_accel = actuator.get_variable_by_name("max_accel")
        step_mode = actuator.get_variable_by_name("step_mode")
        vin_voltage = actuator.get_variable_by_name("vin_voltage")
        gap = (cur_pos_mm + start_gap) / 1000.0  # set gap whether or not test is active

        with open(
            "data/" + csv_name, "a"
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
                start_gap / 1000.0,
                gap,
                sample_volume,
                test_active,
            ]
            dataline = ",".join(map(str, output_params)) + "\n"
            datafile.write(dataline)
            # print(dataline)

        times.append(cur_duration)
        forces.append(force)
        gaps.append(gap)

        sleep(0.02)

        if (time() - start_time) >= 7200 or (
            (not ac.is_alive()) and (time() - start_time) > 1
        ):
            print("Time since started: {:.0f}".format(time() - start_time))
            print("Actuator thread dead? {:}".format(not ac.is_alive()))
            print("end of background")
            break

    print("=" * 20 + " BACKGROUND IS DONE " + "=" * 20)


lc = threading.Thread(name="loadcell", target=load_cell_thread)
ac = threading.Thread(name="actuator", target=actuator_thread)
bkg = threading.Thread(name="background", target=background)

lc.start()
ac.start()
bkg.start()

max_time_window = 30
ax1 = fig.add_subplot(1, 1, 1)
ax2 = ax1.twinx()

color1 = "C0"
color2 = "C1"


def animate(i):
    global ax1, ax2, times, forces, gaps

    if len(times) <= 0:
        return

    ax1.clear()
    ax2.clear()

    timesTemp = times[:]
    forcesTemp = forces[:]
    gapsTemp = gaps[:]

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Force [g]", color=color1)
    ax2.set_ylabel("Gap [mm]", color=color2)

    ax1.plot(timesTemp, forcesTemp, color1, label="Force")
    ax2.plot(timesTemp, [1000 * g for g in gapsTemp], color2, label="Gap")

    plt.xlim(min(timesTemp), max(max(timesTemp), max_time_window))
    plt.title("Sample: {:}".format(sample_str))

    ax1.set_ylim((min(forcesTemp), max(forcesTemp)))
    ax2.set_ylim((0, 1000 * max(gapsTemp)))

    # Color y-ticks
    ax1.tick_params(axis="y", colors=color1)
    ax2.tick_params(axis="y", colors=color2)

    # Color y-axes
    ax1.spines["left"].set_color(color1)
    ax2.spines["left"].set_alpha(0)  # hide second left y axis to show first one
    ax2.spines["right"].set_color(color2)

    ax1.grid(True)
    ax2.grid(False)

    # plt.axis("tight")


ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
