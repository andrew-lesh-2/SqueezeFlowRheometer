"""Performs a sequence of stress-relaxation steps. Moves to a given gap, then waits there to measure
how the force response evolves."""

import threading
from time import sleep, time
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
from squeezeflowrheometer import SqueezeFlowRheometer

targets: list[float] = []
"""Set of target gaps"""

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Get test details from user
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    sfr.sample_volume = SqueezeFlowRheometer.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    # # Get test details from settings file & config file
    # sample_str = settings["sample_str"]
    # start_gap = float(scale.config["gap"])

    first_gap = sfr.sample_volume ** (1.0 / 3.0) * 1000  # mm
    # first_gap = 3
    min_gap = sfr.sample_volume / SqueezeFlowRheometer.HAMMER_AREA * 1000  # mm
    targets = np.geomspace(first_gap, 0.5 * min_gap, 20).tolist()
    sfr.target = targets[0]

    sfr.check_tare()

    sfr.set_max_speed_mms(5)

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    # Set up files/folders
    output_file_name_base = (
        sfr.get_second_date_str()
        + f"_set_gap_squeeze_flow{sample_str}_{round(sfr.sample_volume * 1e6):d}mL"
    )
    sfr.data_file_name = output_file_name_base + "-data.csv"
    sfr.create_figures_folder()
    sfr.create_data_file(
        "Current Time,Elapsed Time,Current Position (mm),Current Position,Target Position,Current Velocity (mm/s),Current Velocity,Target Velocity,Max Speed,Max Decel,Max Accel,Step Mode,Voltage In (mV),Current Force ({:}),Target Force ({:}),Start Gap (m),Current Gap (m),Viscosity (Pa.s),Yield Stress (Pa),Sample Volume (m^3),Viscosity Volume (m^3), Test Active?, Spread beyond hammer?\n".format(
            sfr.units, sfr.units
        )
    )


def actuator_thread():
    """Drives actuator"""

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 2  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(sfr.times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        sfr.times = sfr.times[-keep_datapoints:]
        sfr.forces = sfr.forces[-keep_datapoints:]
        sfr.gaps = sfr.gaps[-keep_datapoints:]
        sfr.yield_stress_guesses = sfr.yield_stress_guesses[-keep_datapoints:]

    step_id = 0
    sfr.target = targets[step_id]

    sfr.test_active = True

    step_rest_length = (
        45  # seconds, how long to sit at the current spot before moving on
    )
    max_strain_rate = 0.05  # 1/s
    for t in targets:
        print(f"Target gap is {t:.2f}mm")

        target_pos = t - sfr.start_gap
        sfr.target = t

        max_speed = max_strain_rate * t
        sfr.set_max_speed_mms(max_speed)

        sfr.heartbeat()
        sfr.move_to_mm(target_pos)
        sfr.heartbeat()
        print("Reached position, waiting")

        step_rest_start_time = time()
        while time() - step_rest_start_time <= step_rest_length:
            sleep(0.1)
            sfr.heartbeat()

    print("Last step complete. Test is done.")
    sfr.test_active = False
    sfr.save_figure(fig)

    # Gradually back off of material since it's probably also stiff to retract from
    for t in reversed(targets):
        target_pos = t - sfr.start_gap

        max_speed = max_strain_rate * t
        sfr.set_max_speed_mms(max_speed)

        sfr.heartbeat()
        sfr.move_to_mm(target_pos)
        sfr.heartbeat()

    sfr.set_max_speed_mms(5)
    sfr.go_home_quiet_down()
    print("Actuator thread is done.")


sfr.load_cell_thread = threading.Thread(
    name="loadcell", target=sfr.load_cell_thread_method
)
sfr.actuator_thread = threading.Thread(name="actuator", target=actuator_thread)
sfr.data_writing_thread = threading.Thread(
    name="background", target=sfr.data_writing_thread_method
)

sfr.load_cell_thread.start()
sfr.actuator_thread.start()
sfr.data_writing_thread.start()

MAX_TIME_WINDOW = 30
ax1 = fig.add_subplot(1, 1, 1)
ax2 = ax1.twinx()
ax3 = ax1.twinx()

COLOR1 = "C0"
COLOR2 = "C1"
COLOR3 = "C2"


def animate(_):
    """Plot data throughout the test"""

    if len(sfr.times) <= 0:
        return

    ax1.clear()
    ax2.clear()
    ax3.clear()

    # Throw away data & timestamps that are too old.
    # while times[-1] - times[0] > max_time_window:
    #     times.pop(0)
    #     forces.pop(0)
    #     gaps.pop(0)

    times_temp = sfr.times[:]
    forces_temp = sfr.forces[:]
    gaps_temp = sfr.gaps[:]
    yield_stress_guesses_temp = sfr.yield_stress_guesses[:]

    # print("{:7d}: {:}".format(len(timesTemp), timesTemp[-1] - timesTemp[0]))

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Force [g]", color=COLOR1)
    ax2.set_ylabel("Gap [mm]", color=COLOR2)
    ax3.set_ylabel("Yield Stress [Pa]", color=COLOR3)

    ax1.plot(times_temp, forces_temp, COLOR1, label="Force")
    ax2.plot(times_temp, [1000 * g for g in gaps_temp], COLOR2, label="Gap")
    ax3.plot(times_temp, yield_stress_guesses_temp, COLOR3, label="Yield Stress")

    plt.xlim(min(times_temp), max(*times_temp, MAX_TIME_WINDOW))
    plt.title(f"Sample: {sample_str}")

    # ax1.set_ylim((-0.5, max(2 * sfr.target, max(forcesTemp))))
    ax2.set_ylim((0, 1000 * max(gaps_temp)))

    # Color y-ticks
    ax1.tick_params(axis="y", colors=COLOR1)
    ax2.tick_params(axis="y", colors=COLOR2)
    ax3.tick_params(axis="y", colors=COLOR3)

    # Color y-axes
    ax1.spines["left"].set_color(COLOR1)
    ax2.spines["left"].set_alpha(0)  # hide second left y axis to show first one
    ax2.spines["right"].set_color(COLOR2)
    ax3.spines["left"].set_alpha(0)  # hide third left y axis to show first one
    ax3.spines["right"].set_color(COLOR3)

    # ax3.spines["right"].set_position(
    #     ("axes", 1.1)
    # )  # move it further right to prevent overlap
    ax3.spines["right"].set_position(
        ("outward", 10)
    )  # move it further right to prevent overlap

    ax1.grid(True)
    ax2.grid(False)
    ax3.grid(False)

    # plt.axis("tight")


ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
