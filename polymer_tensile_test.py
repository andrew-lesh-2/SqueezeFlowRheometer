"""Plots force while slowly advancing to target gap (used for tensile testing of polymer bridges)"""

import threading
from time import sleep, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Get test details from user
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    sfr.step_duration = SqueezeFlowRheometer.input_step_duration(sfr.default_duration)
    sfr.sample_volume = SqueezeFlowRheometer.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    target_gap_line = input("Enter the target gap in [mm]: ")
    target_gap = SqueezeFlowRheometer.find_num_in_str(target_gap_line)
    """Target gap for polymer stretching"""

    sfr.check_tare()
    sfr.set_max_speed_mms(0.01)  # Set maximum speed to a very slow 0.01 mm/s

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    output_file_name_base = (
        sfr.get_second_date_str()
        + f"_polymer_stretch_test_{sample_str}_"
        + f"{round(sfr.sample_volume * 1e6):d}mL_{round(target_gap):d}mm"
    )

    sfr.data_file_name = output_file_name_base + "-data.csv"
    sfr.create_figures_folder()
    sfr.create_data_file(
        "Current Time,Elapsed Time,Current Position (mm),Current Position,Target Position,"
        + "Current Velocity (mm/s),Current Velocity,Target Velocity,Max Speed,Max Decel,Max Accel,"
        + f"Step Mode,Voltage In (mV),Current Force ({sfr.units:}),Target Force ({sfr.units:}),"
        + "Start Gap (m),Current Gap (m),Viscosity (Pa.s),Yield Stress (Pa),Sample Volume (m^3),"
        + "Viscosity Volume (m^3), Test Active?, Spread beyond hammer?\n"
    )

def actuator_thread():
    """Drives actuator"""

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 0  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(sfr.times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        sfr.times = sfr.times[-keep_datapoints:]
        sfr.forces = sfr.forces[-keep_datapoints:]
        sfr.gaps = sfr.gaps[-keep_datapoints:]

    sfr.test_active = True

    # Move from start position to target gap slowly
    print(f"Target gap is {target_gap:.2f}mm")
    target_pos = target_gap - sfr.start_gap

    sfr.move_to_mm(target_pos)  # Move continuously to the target gap
    sfr.heartbeat()

    while sfr.get_current_position_mms() != target_pos:
        sleep(0.1)  # Check the position every 0.1 seconds
        sfr.heartbeat()

    print("Reached position, waiting")

    rest_start_time = time()
    while time() - rest_start_time <= sfr.step_duration:
        sleep(0.1)
        sfr.heartbeat()

    print("Test is done.")
    sfr.test_active = False

    sfr.save_figure(fig)
    print("Actuator thread is done.")

sfr.load_cell_thread = threading.Thread(
    name="loadcell", target=sfr.load_cell_thread_method, args=[False]
)
sfr.actuator_thread = threading.Thread(name="actuator", target=actuator_thread)
sfr.data_writing_thread = threading.Thread(
    name="background", target=sfr.data_writing_thread_method, args=[False]
)

sfr.load_cell_thread.start()
sfr.actuator_thread.start()
sfr.data_writing_thread.start()

MAX_TIME_WINDOW = 30
ax1 = fig.add_subplot(1, 1, 1)
ax2 = ax1.twinx()

COLOR1 = "C0"
COLOR2 = "C1"

def animate(_):
    """Plot data throughout the test"""

    if len(sfr.times) <= 0:
        return

    ax1.clear()
    ax2.clear()

    times_temp = sfr.times[:]
    forces_temp = sfr.forces[:]
    gaps_temp = sfr.gaps[:]

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Force [g]", color=COLOR1)
    ax2.set_ylabel("Gap [mm]", color=COLOR2)

    ax1.plot(times_temp, forces_temp, COLOR1, label="Force")
    ax2.plot(times_temp, [1000 * g for g in gaps_temp], COLOR2, label="Gap")

    plt.xlim(min(times_temp), max(*times_temp, MAX_TIME_WINDOW))
    plt.title(f"Sample: {sample_str:}")

    ax1.set_ylim((min(forces_temp), max(forces_temp)))
    ax2.set_ylim((0, 1000 * max(gaps_temp)))

    # Color y-ticks
    ax1.tick_params(axis="y", colors=COLOR1)
    ax2.tick_params(axis="y", colors=COLOR2)

    # Color y-axes
    ax1.spines["left"].set_color(COLOR1)
    ax2.spines["left"].set_alpha(0)  # hide second left y axis to show first one
    ax2.spines["right"].set_color(COLOR2)

    ax1.grid(True)
    ax2.grid(False)

ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()