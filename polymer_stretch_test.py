import threading
from time import sleep, time
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Input test values from external settings file
    settings_path = "test_settings.json"
    with open(settings_path, "r") as read_file:
        settings = json.load(read_file)

    # Get test details from user
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    test_duration = SqueezeFlowRheometer.input_step_duration(sfr.default_duration)
    sfr.sample_volume = SqueezeFlowRheometer.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    target_gap_line = input("Enter the target gap in [mm]: ")
    target_gap = SqueezeFlowRheometer.find_num_in_str(target_gap_line)
    """Target gap for polymer stretching"""

    # # Get test details from settings file & config file
    # sample_str = settings["sample_str"]
    # start_gap = float(sfr.config["gap"])

    sfr.check_tare()
    sfr.set_max_speed_mms(5)

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
    global sfr, fig

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 0  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        times = times[-keep_datapoints:]
        forces = forces[-keep_datapoints:]
        gaps = gaps[-keep_datapoints:]

    sfr.test_active = True

    # Move from start position to target gap
    print("Target gap is {:.2f}mm".format(target_gap))
    target_pos = target_gap - sfr.start_gap
    sfr.heartbeat()
    sfr.move_to_mm(target_pos)
    sfr.heartbeat()
    print("Reached position, waiting")

    rest_start_time = time()
    while time() - rest_start_time <= test_duration:
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

max_time_window = 30
ax1 = fig.add_subplot(1, 1, 1)
ax2 = ax1.twinx()

color1 = "C0"
color2 = "C1"


def animate(i):
    global ax1, ax2, sfr

    if len(sfr.times) <= 0:
        return

    ax1.clear()
    ax2.clear()

    timesTemp = sfr.times[:]
    forcesTemp = sfr.forces[:]
    gapsTemp = sfr.gaps[:]

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
