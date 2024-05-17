import pytic
from time import sleep


def move_to_pos(pos):
    tic.set_target_position(pos)
    while tic.variables.current_position != tic.variables.target_position:
        sleep(0.1)
        tic.reset_command_timeout()


if __name__ == "__main__":
    tic = pytic.PyTic()

    # Connect to first available Tic Device serial number over USB
    serial_nums = tic.list_connected_device_serial_numbers()

    # print(serial_nums)
    tic.connect_to_serial_number(serial_nums[0])

    tic.set_max_decel(500000)
    tic.set_max_accel(500000)

    # Load configuration file and apply settings
    # tic.settings.load_config('actuator_test_1_config.yml')
    # tic.settings.apply()

    # - Motion Command Sequence ----------------------------------

    # Actually don't do this, I want you to go back home with this.
    # # Zero current motor position
    # tic.halt_and_set_position(0)

    # Energize Motor
    print("energizing")
    tic.energize()
    print("exiting safe start")
    tic.exit_safe_start()

    print("Going to zero")
    move_to_pos(0)

    # De-energize motor and get error status
    print("entering safe start")
    tic.enter_safe_start()
    print("deenergizing")
    tic.deenergize()
    print(tic.variables.error_status)

    print("=" * 20 + " FOREGROUND IS DONE " + "=" * 20)
