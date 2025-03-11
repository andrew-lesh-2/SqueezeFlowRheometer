from time import sleep
from squeezeflowrheometer import SqueezeFlowRheometer

if __name__ == "__main__":
    sfr = SqueezeFlowRheometer()

    # throw away dummy input
    input()

    # Get test details from user
    sfr.start_gap = SqueezeFlowRheometer.input_start_gap(sfr)

    # Zero current motor position
    sfr.halt_and_set_position(0)
    sfr.heartbeat()

    # Pause
    print("Waiting 2 seconds before starting")
    sleep(2)

    # - Motion Command Sequence ----------------------------------
    sfr.startup()

    # Move from start position to target gap
    print("Moving to zero gap.")
    target_pos = sfr.start_gap

    # Stop a little bit away
    first_stop = min(0, target_pos + 0.5)
    sfr.heartbeat()
    sfr.move_to_mm(first_stop)
    sfr.heartbeat()

    # Go to final point slowly
    sfr.set_max_speed_mms(0.1)
    sfr.heartbeat()
    sfr.move_to_mm(target_pos)
    sfr.heartbeat()
    print("Reached position, done.")
    print("Actuator thread is done.")
