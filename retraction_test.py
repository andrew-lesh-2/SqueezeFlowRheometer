import threading
from time import sleep
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer


if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Input test values from external settings file
    settings_path = "Retraction/retraction_settings.json"
    with open(settings_path, "r") as read_file:
        settings = json.load(read_file)

    # Get test details from user
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    sfr.sample_volume = SqueezeFlowRheometer.input_sample_volume()
    approach_gap = SqueezeFlowRheometer.input_retract_start_gap(
        sfr.sample_volume, settings
    )
    retract_speed = SqueezeFlowRheometer.input_retract_speed(settings)
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    # # Get test details from settings file & config file
    # start_gap = float(sfr.config["gap"])
    # approach_gap = float(settings["retract_gap_mm"])
    # retract_speed = float(settings["retract_speed_mms"])
    # sample_str = float(settings["sample_str"])

    sfr.check_tare()

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    output_file_name_base = (
        sfr.get_day_date_str()
        + "_"
        + "retraction_{:}_{:d}mL_{:d}mm_{:d}mms".format(
            sample_str,
            round(sfr.sample_volume * 1e6),
            round(approach_gap),
            round(retract_speed),
        )
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
    global test_active, times, gaps, forces

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    approach_velocity = settings["approach_max_speed_mms"]

    # Start by approaching and waiting until force is non-negligible
    sfr.set_max_speed_mms(approach_velocity)

    retract_start_pos = -sfr.start_gap + approach_gap
    print("Moving to start position of {:.1f}mm".format(retract_start_pos))
    sfr.move_to_mm(retract_start_pos)

    sfr.heartbeat()

    pause_time = settings["pause_time_at_gap"]
    print("Reached start position. Pausing for {:d} second(s).".format(pause_time))

    sfr.set_max_speed_mms(retract_speed)
    for i in range(0, int(pause_time / 0.1)):
        sfr.heartbeat()
        sleep(0.1)
        sfr.heartbeat()

    # Now that test is about to start, throw away most of the pre-test data.
    data_keep_time = 2  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(sfr.times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        sfr.times = sfr.times[-keep_datapoints:]
        sfr.forces = sfr.forces[-keep_datapoints:]
        sfr.gaps = sfr.gaps[-keep_datapoints:]

    sfr.heartbeat()

    # Start the test
    print("Starting retraction.")
    test_active = True
    print(sfr.get_pos_mm())
    sfr.move_to_mm(0)
    print(sfr.get_pos_mm())
    test_active = False
    print("Retraction complete.")


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
fig = plt.figure(figsize=(7.2, 4.8))
ax1 = fig.add_subplot(1, 1, 1)
ax2 = ax1.twinx()

color1 = "C0"
color2 = "C1"
color3 = "C2"


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
