import threading
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Get test details from user
    start_gap = SqueezeFlowRheometer.input_start_gap(sfr)

    sfr.check_tare()

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    output_file_name_base = sfr.get_day_date_str() + "_" + "rigidity_test"

    sfr.data_file_name = output_file_name_base + "-data.csv"

    sfr.create_figures_folder()

    sfr.create_data_file(
        "Current Time,Elapsed Time,Current Position (mm),Current Position,Target Position,Current Velocity (mm/s),Current Velocity,Target Velocity,Max Speed,Max Decel,Max Accel,Step Mode,Voltage In (mV),Current Force ({:}),Target Force ({:}),Start Gap (m),Current Gap (m),Viscosity (Pa.s),Yield Stress (Pa),Sample Volume (m^3),Viscosity Volume (m^3), Test Active?, Spread beyond hammer?\n".format(
            sfr.units, sfr.units
        )
    )


def actuator_thread():
    """Drives actuator"""
    global sfr, fig

    print("Waiting 2 seconds before starting")
    sleep(2)

    sfr.startup()

    approach_velocity = -0.002  # mm/s, speed to perform rigidity test at
    backoff_dist = 0.2

    # Move to just above the plate
    print("Approaching start point")
    sfr.move_to_mm(-abs(start_gap - backoff_dist))
    print("Reached start point. Now approaching the plate slowly.")

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 0.1  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = int(data_keep_time * data_rate)  # how many datapoints to keep
    if len(times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        times = times[-keep_datapoints:]
        forces = forces[-keep_datapoints:]
        gaps = gaps[-keep_datapoints:]

    sfr.test_active = True

    sfr.set_vel_mms(approach_velocity)
    while abs(sfr.force) < sfr.force_limit:
        sfr.heartbeat()
        gap_mm = sfr.get_pos_mm() + start_gap
        out_str = "F = {:7.3f}{:}, pos = {:8.3f}mm".format(sfr.force, sfr.units, gap_mm)
        print(out_str)

    sfr.test_active = False
    sfr.set_vel_mms(0)

    sfr.set_max_speed_mms(5)
    sfr.end_test(fig)


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

ax1 = fig.add_subplot(1, 1, 1)

color1 = "C0"
color2 = "C1"


def animate(i):
    global ax1, sfr

    if len(sfr.times) <= 0:
        return

    ax1.clear()

    forcesTemp = sfr.forces[:]
    gapsTemp = sfr.gaps[:]

    ax1.set_xlabel("Distance past zero-point [mm]")
    ax1.set_ylabel("Force [g]", color=color1)

    ax1.plot([-1000 * h for h in gapsTemp], forcesTemp, color1, label="Force")

    plt.xlim((-1000 * max(gapsTemp), -1000 * min(gapsTemp)))
    plt.title("Rigidity Test")

    # ax1.set_ylim((-0.5, max(forcesTemp)))

    # Color y-ticks
    ax1.tick_params(axis="y", colors=color1)

    # Color y-axes
    # ax1.spines["left"].set_color(color1)

    ax1.grid(True)

    # plt.axis("tight")


ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
