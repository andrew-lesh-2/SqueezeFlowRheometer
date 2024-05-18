import threading
from time import sleep, time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Get test details from user
    min_gap = SqueezeFlowRheometer.find_num_in_str(
        input("What's the target minimum gap in mm? ")
    )
    start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    test_duration = SqueezeFlowRheometer.input_step_duration(sfr.default_duration)
    sample_volume = SqueezeFlowRheometer.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    # # Get test details from settings file & config file
    # test_duration = settings["test_duration"]
    # sample_str = settings["sample_str"]
    # start_gap = float(scale.config["gap"])

    sfr.check_tare()

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    output_file_name_base = (
        sfr.get_day_date_str()
        + "_"
        + "constant_strain_rate_squeeze_flow_{:}_{:d}mL_{:}mm".format(
            sample_str, round(sample_volume * 1e6), min_gap
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
    # global gap, eta_guess, error, int_error, der_error, sample_volume, test_active, spread_beyond_hammer, visc_volume, yield_stress_guess, times, gaps, forces
    global sfr, fig

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    approach_velocity = -0.5  # mm/s, speed to approach bath of fluid at
    force_threshold = 0.6  # g, force must exceed this for control system to kick in.

    # Start by approaching and waiting until force is non-negligible
    sfr.set_vel_mms(approach_velocity)
    while True:
        sfr.heartbeat()
        if abs(sfr.force) > sfr.force_limit:
            sfr.end_test(fig)
            break
        if sfr.force > force_threshold:
            sfr.test_active = True
            break
        if abs(sfr.get_pos_mm()) >= start_gap:
            print("Hit the hard-stop without ever exceeding threshold force, stopping.")
            sfr.end_test(fig)
            return

    print("Force threshold met, switching over to constant strain rate.")
    initial_gap = start_gap + sfr.get_pos_mm()

    print(initial_gap)
    print(min_gap)
    strain_rate = math.log(initial_gap / min_gap) / test_duration

    # Now that test is active, throw away most of the pre-test data.
    data_keep_time = 2  # how many seconds to keep
    data_rate = 10  # roughly how many datapoints I record per second
    keep_datapoints = data_keep_time * data_rate  # how many datapoints to keep
    if len(times) > keep_datapoints:
        # only throw away points if they're older than data_keep_time
        times = times[-keep_datapoints:]
        forces = forces[-keep_datapoints:]
        gaps = gaps[-keep_datapoints:]
        yieldStressGuesses = yieldStressGuesses[-keep_datapoints:]

    gap_m = (sfr.get_pos_mm() + start_gap) / 1000.0  # current gap in m
    ref_gap = max(
        ref_gap, gap_m
    )  # if sample volume is large, might need to increase the ref gap
    v = 0

    while True:
        # Check if force beyond max amount
        if abs(sfr.force) > sfr.force_limit:
            print(
                "Force was too large, stopping - {:3.2f}{:}".format(
                    sfr.force, sfr.units
                )
            )
            sfr.end_test(fig)
            return

        # Check if went too far
        cur_pos_mm = sfr.get_pos_mm()
        gap_m = (cur_pos_mm + start_gap) / 1000.0  # current gap in m
        if cur_pos_mm >= start_gap:
            print("Hit the hard-stop, stopping.")
            sfr.end_test(fig)
            return

        # Check if returned towards zero too far
        if abs(cur_pos_mm) <= 1:
            print("Returned too close to home, stopping.")
            sfr.end_test(fig)
            return

        if time() - sfr.start_time >= test_duration or (1000 * gap_m) <= min_gap:
            print("Test complete, stopping.")
            sfr.end_test(fig)
            return

        v = -(1000 * gap_m) * strain_rate
        sfr.set_vel_mms(v)

        out_str = "{:6.2f}{:}, gap = {:6.2f}, v = {:11.5f}, strain rate = {:}".format(
            sfr.force, sfr.units, gap_m * 1000, v, strain_rate
        )
        print(out_str)

        sfr.heartbeat()


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
ax3 = ax1.twinx()

color1 = "C0"
color2 = "C1"
color3 = "C2"


def animate(i):
    global ax1, ax2, ax3, sfr

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

    timesTemp = sfr.times[:]
    forcesTemp = sfr.forces[:]
    gapsTemp = sfr.gaps[:]
    yieldStressGuessesTemp = sfr.yieldStressGuesses[:]

    # print("{:7d}: {:}".format(len(timesTemp), timesTemp[-1] - timesTemp[0]))

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Force [g]", color=color1)
    ax2.set_ylabel("Gap [mm]", color=color2)
    ax3.set_ylabel("Yield Stress [Pa]", color=color3)

    ax1.plot(timesTemp, forcesTemp, color1, label="Force")
    ax2.plot(timesTemp, [1000 * g for g in gapsTemp], color2, label="Gap")
    ax3.plot(timesTemp, yieldStressGuessesTemp, color3, label="Yield Stress")

    plt.xlim(min(timesTemp), max(max(timesTemp), max_time_window))
    plt.title("Sample: {:}".format(sample_str))

    # ax1.set_ylim((-0.5, max(2 * target, max(forcesTemp))))
    ax2.set_ylim((0, 1000 * max(gapsTemp)))

    # Color y-ticks
    ax1.tick_params(axis="y", colors=color1)
    ax2.tick_params(axis="y", colors=color2)
    ax3.tick_params(axis="y", colors=color3)

    # Color y-axes
    ax1.spines["left"].set_color(color1)
    ax2.spines["left"].set_alpha(0)  # hide second left y axis to show first one
    ax2.spines["right"].set_color(color2)
    ax3.spines["left"].set_alpha(0)  # hide third left y axis to show first one
    ax3.spines["right"].set_color(color3)

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
