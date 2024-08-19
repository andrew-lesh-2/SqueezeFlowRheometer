"""Performs a squeeze flow test attempting to maintain a target force from a given sequence of
targets. Maintains each target force for a given duration, then moves to the next target."""

import threading
from time import sleep, time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from squeezeflowrheometer import SqueezeFlowRheometer

# - Initialization -------------------------------------------

targets = []
"""Monotonically increasing list of target forces.
Will run a test on each force, one after the other."""

fig = plt.figure(figsize=(7.2, 4.8))

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # Get test details from user
    targets = SqueezeFlowRheometer.input_targets(sfr.units, sfr.test_settings)
    sfr.target = targets[0]  # start at first step
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)
    sfr.step_duration = SqueezeFlowRheometer.input_step_duration(sfr.default_duration)
    sfr.sample_volume = SqueezeFlowRheometer.input_sample_volume()
    sample_str = input("What's the sample made of? This will be used for file naming. ")

    # # Get test details from settings file & config file
    # targets = settings["targets"]
    # sfr.step_duration = settings["test_duration"]
    # sample_str = settings["sample_str"]
    # start_gap = float(scale.config["gap"])

    sfr.check_tare()

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    output_file_name_base = (
        sfr.get_second_date_str()
        + f"_sfrObjectTestPID_{sample_str}_{round(sfr.sample_volume * 1e6):d}mL"
    )

    sfr.data_file_name = output_file_name_base + "-data.csv"

    sfr.create_figures_folder()

    sfr.create_data_file(
        "Current Time,Elapsed Time,Current Position (mm),Current Position,Target Position,"
        + "Current Velocity (mm/s),Current Velocity,Target Velocity,Max Speed,Max Decel,Max Accel,"
        + f"Step Mode,Voltage In (mV),Current Force ({sfr.units}),Target Force ({sfr.units}),"
        + "Start Gap (m),Current Gap (m),Viscosity (Pa.s),Yield Stress (Pa),Sample Volume (m^3),"
        + "Viscosity Volume (m^3),Test Active?,Spread beyond hammer?,Error,K_P,Integrated Error,"
        + "K_I,Error Derivative,K_D\n"
    )


def actuator_thread():
    """Drives actuator"""

    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------

    sfr.startup()

    approach_velocity = -0.5  # mm/s, speed to approach bath of fluid at
    force_threshold = 0.5  # g, force must exceed this for control system to kick in.

    # Start by approaching and waiting until force is non-negligible
    sfr.set_vel_mms(approach_velocity)
    while True:
        sfr.heartbeat()
        if abs(sfr.force) > sfr.force_limit:
            sfr.end_test(fig)
            return
        if sfr.force > force_threshold:
            sfr.test_active = True
            break
        if abs(sfr.get_pos_mm()) >= sfr.start_gap:
            print("Hit the hard-stop without ever exceeding threshold force, stopping.")
            sfr.end_test(fig)
            return

    # reset integrated error - prevent integral windup
    sfr.int_error = 0

    print("Force threshold met, switching over to PID control.")

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

    step_start_time = time()
    step_id = 0  # Which target step the test is currently on. 0 is the first step
    sfr.target = targets[step_id]
    step_increase = (
        sfr.target
    )  # how much the target force has increased from the last step

    gap_m = (sfr.get_pos_mm() + sfr.start_gap) / 1000.0  # current gap in m
    sfr.ref_gap = max(
        sfr.ref_gap, gap_m
    )  # if sample volume is large, might need to increase the ref gap

    mute_derivative_term_steps = 0
    mute_derivative_term_steps_max = 100
    while True:
        # Check if force beyond max amount
        if abs(sfr.force) > sfr.force_limit:
            print(f"Force was too large, stopping - {sfr.force:3.2f}{sfr.units}")
            sfr.end_test(fig)
            return

        # Check if went too far
        cur_pos_mm = sfr.get_pos_mm()
        gap_m = (cur_pos_mm + sfr.start_gap) / 1000.0  # current gap in m
        if cur_pos_mm >= sfr.start_gap:
            print("Hit the hard-stop, stopping.")
            sfr.end_test(fig)
            return

        # Check if returned towards zero too far
        if abs(cur_pos_mm) <= 1:
            print("Returned too close to home, stopping.")
            sfr.end_test(fig)
            return

        if time() - step_start_time >= sfr.step_duration:
            step_id = step_id + 1
            if step_id < len(targets):
                print("Step time limit reached, next step.")
                sfr.target = targets[step_id]
                step_increase = targets[step_id] - targets[step_id - 1]
                step_start_time = time()
                mute_derivative_term_steps = mute_derivative_term_steps_max
            else:
                print("Last step complete. Test is done.")
                sfr.end_test(fig)
                return

        # Prevent integral windup
        int_threshold = 10
        if abs(sfr.int_error) > int_threshold:
            sfr.int_error = math.copysign(int_threshold, sfr.int_error)

        # vel_P = -K_P * error
        vel_P = (
            -sfr.variable_K_P(sfr.error, step_increase) * sfr.error
        )  # Proportional component of velocity response
        vel_I = -sfr.K_I * sfr.int_error  # Integral component of velocity response
        vel_D = -sfr.K_D * sfr.der_error  # Derivative component of velocity response

        if mute_derivative_term_steps > 0:
            mute_derivative_term_steps = mute_derivative_term_steps - 1
            sfr.der_error = 0
            vel_D = 0

        # v_new = vel_P + vel_D + vel_I # modified PID control
        v_new = vel_P + vel_I  # modified PI control
        v_new = (
            v_new * (gap_m / sfr.ref_gap) ** 2
        )  # slow the response as the gap gets thinner
        # v_new = min(v_new, 0)  # Only go downward
        sfr.set_vel_mms(v_new)

        out_str = (
            f"{sfr.force:6.2f}{sfr.units}, err = {sfr.error:6.2f}, "
            + f"errI = {sfr.int_error:6.2f}, errD = {sfr.der_error:7.2f}, "
            + f"gap = {gap_m * 1000:6.2f}mm, v = {v_new:11.5f} : vP = {vel_P:6.2f}, "
            + f"vI = {vel_I:6.2f}, vD = {vel_D:6.2f}"
        )
        print(out_str)

        sfr.heartbeat()


sfr.load_cell_thread = threading.Thread(
    name="loadcell", target=sfr.load_cell_thread_method, args=[True]
)
sfr.actuator_thread = threading.Thread(name="actuator", target=actuator_thread)
sfr.data_writing_thread = threading.Thread(
    name="background", target=sfr.data_writing_thread_method, args=[True]
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

    times_temp = sfr.times[:]
    forces_temp = sfr.forces[:]
    gaps_temp = sfr.gaps[:]
    yield_stress_guesses_temp = sfr.yield_stress_guesses[:]

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
