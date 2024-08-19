"""Provides SqueezeFlowRheometer class which combines operations of OpenScale
and TicActuator classes to operate load cell and actuator from one object"""

import re
import math
from pathlib import Path
from datetime import datetime
import threading
from time import sleep, time
import json
import os
from matplotlib.figure import Figure
from LoadCell.openscale import OpenScale
from Actuator.ticactuator import TicActuator


class SqueezeFlowRheometer(OpenScale, TicActuator):
    """Combines operations of OpenScale
    and TicActuator classes to operate load cell and actuator from one object"""

    HAMMER_RADIUS = 25e-3
    """Radius of circular top plate in m"""
    HAMMER_AREA = math.pi * HAMMER_RADIUS**2
    """Area of circular top plate in m^2"""
    FORCE_UP_SIGN = 1
    """Sign of a positive force. This should be 1 or -1, and is
    used to compute velocity based on force"""
    MAX_TEST_DURATION = 7200
    """Max test duration in seconds. Once the test is this long, it will end."""
    DEFAULT_STEP_DURATION = 300
    """Default length of a test step in seconds"""

    def __init__(self):

        ## Test info and instrument settings
        self.settings_path: str = "test_settings.json"
        self.date: datetime = datetime.now()
        self.test_settings: dict = {}
        self.default_duration: float = 0
        """Default length of a test in seconds, set by test settings json file. User may simply
        accept the default instead of choosing it themselves"""
        self.step_duration: float = 0
        """Selected length of test in seconds"""
        self.figure_name: str = ""
        """File name for figure"""
        self.figure_path: str = ""
        """File path for saved figure"""

        ## Threads for simultaneous actuator control, load cell reading, and data writing
        self.actuator_thread: threading.Thread
        self.load_cell_thread: threading.Thread
        self.data_writing_thread: threading.Thread

        ## Important experimental values
        self.start_gap: float = 0
        """Initial gap at the start of the test in mm"""
        self.test_active: bool = False
        """Whether or not the test is currently running, or just on approach/retract"""
        self.sample_volume: float = 0
        """The amount of sample present"""
        self.start_time: float = 0
        """When the test started, in unix time, measured in seconds"""
        self.cur_time: float = 0
        """Current time, in unix time, measured in seconds"""
        self.cur_duration: float = 0
        """How long the test has been going on"""
        self.force: float = 0
        """The current force in g"""
        self.target: float = 0
        """The current target value. For constant-force tests, this is a force in grams.
        For set-gap tests, this is the target gap in mm"""
        self.gap: float = 0
        """The current gap in m"""
        self.eta_guess: float = 0
        """A guess at the fluid's Newtonian viscosity"""
        self.yield_stress_guess: float = 0
        """A guess at the fluid's yield stress. May assume no slip or perfect slip
        depending on code."""
        self.visc_volume: float = 0
        """The amount of sample that remains between the plates. Used to recognize squeezeout"""
        self.spread_beyond_hammer: bool = False
        """Whether the sample has been squeezed past the bounds of the plates"""

        ## PID control
        self.a: float = 0
        """Parameter for variable K_P"""
        self.b: float = 0
        """Parameter for variable K_P"""
        self.c: float = 0
        """Parameter for variable K_P"""
        self.d: float = 0
        """Parameter for variable K_P"""
        self.variable_KP = lambda er, tar: (self.a + self.b) / 2 + (
            self.a - self.b
        ) / 2 * math.tanh(self.c * ((er / tar) ** 2 - self.d))
        """Given the current error and the current target force, get a proportional control
        parameter. This is a function instead of a constant because this was found to work
        better due to the nonlinear nature of squeeze flow."""
        self.ref_gap: float = 1
        """Parameter used to slow controller response as gap gets smaller. Defaulted to 1 to
        prevent divide-by-zero error"""
        self.error: float = 0
        """How far from the target value the force is. For PID Loop"""
        self.K_P: float = 0
        """Proportional coefficient for PID loop"""
        self.int_error: float = 0
        """Integrated error for PID loop."""
        self.K_I: float = 0
        """Integral coefficient for PID loop"""
        self.der_error: float = 0
        """Derivative error for PID loop."""
        self.K_D: float = 0
        """Derivative coefficient for PID loop"""

        ## Plotting values
        self.times: list[float] = []
        """List of time points for the test thus far. Used to liveplot test data"""
        self.forces: list[float] = []
        """List of force values for the test thus far. Used to liveplot test data"""
        self.gaps: list[float] = []
        """List of gap values for the test thus far. Used to liveplot test data"""
        self.yield_stress_guesses: list[float] = []
        """List of computed yield stress values for the test thus far. Used to liveplot test data"""

        ## Initialize load cell reading
        OpenScale.__init__(self)
        self.load_settings()

        ## Initialize actuator controller
        TicActuator.__init__(self, step_mode=self.test_settings["actuator_step_mode"])
        self.set_max_accel_mmss(self.test_settings["actuator_max_accel_mmss"], True)
        self.set_max_speed_mms(self.test_settings["actuator_max_speed_mms"])

        ## Data saving details
        if "data_path" in self.test_settings:
            self.data_folder: str = os.path.join(
                self.test_settings["data_path"], "data"
            )
        else:
            self.data_folder: str = "data"
        self.data_file_name: str
        self.figure_folder = self.get_figure_folder_path()

    @staticmethod
    def input_targets(scale_unit: str, settings: dict) -> list[float]:
        """Takes in a list of strictly increasing force targets from the user

        Args:
            inp (str): units of the scale calibration to get appropriate user input

        Returns:
            list[float]: list of force targets to run squeeze flow to
        """
        target_list_acceptable = False

        while not target_list_acceptable:
            # Take in values
            targets_string = input(
                (
                    f"Please give the set of force targets in [{scale_unit}] you want to test, "
                    "separated by commas and/or spaces, or press enter to use the default values: "
                )
            )
            if "config" in targets_string.lower() or len(targets_string) <= 0:
                targets_list = settings["targets"]
            else:
                targets_string = targets_string.replace(
                    " ", ","
                )  # convert spaces to commas to ensure succesful separation of values
                targets_str_list = targets_string.split(",")  # split list at commas
                targets_str_list = list(
                    filter(None, targets_str_list)
                )  # throw out any empty strings created by a
                # ", " becoming ",," being split into an empty string
                targets_list = [
                    float(tar) for tar in targets_str_list
                ]  # parse string to float
            target_list_acceptable = True  # default to assuming the values are fine

            # Catch if it's not strictly increasing
            increasing_bools = [
                targets_list[i] < targets_list[i + 1]
                for i in range(len(targets_list) - 1)
            ]
            strictly_increasing = all(increasing_bools)
            if not strictly_increasing:
                print(
                    "The set of forces must be strictly increasing. Every"
                    " force target must be higher than the previous value."
                )
                target_list_acceptable = False
            print()

        targets_list_printable = ", ".join([str(tar) for tar in targets_list])
        print(
            f"The list of target forces in [{scale_unit}] is {targets_list_printable}"
        )
        return targets_list

    @staticmethod
    def find_num_in_str(inp: str) -> float:
        """Finds a number in a string potentially containing additional exraneous text

        Args:
            inp (str): input string that contains a number and could have extra whitespace, punctuation, or other

        Returns:
            float: the number contained therein, now parsed as a positive float
        """
        temp = re.compile("[0-9.]+")
        res = temp.search(inp).group(0)
        return abs(float(res))

    @staticmethod
    def input_start_gap(scale) -> float:
        """Gets start gap in mm from user.

        Returns:
            float: the start gap in mm
        """
        gap_line, gap = SqueezeFlowRheometer.get_user_input(
            "Enter the current gap in [mm]. If you want to use the gap in the config file, just hit Enter: "
        )
        if "config" in gap_line.lower() or len(gap_line) <= 0:
            gap = float(scale.config["gap"])

        print(f"Starting gap is {gap:.2f}mm")
        return gap

    @staticmethod
    def input_sample_volume() -> float:
        """Gets sample volume in mL from user

        Returns:
            float: sample volume in mL
        """
        sample_vol = SqueezeFlowRheometer.get_user_input(
            "Enter the sample volume in [mL]: "
        )[
            1
        ]  # just get the value, ignore the raw response
        sample_vol = sample_vol * 1e-6  # m^3
        print(f"Sample volume is {(sample_vol * 1e6):.2f}mL")
        return sample_vol

    @staticmethod
    def get_user_input(input_prompt: str) -> tuple[str, float]:
        """Prompt user with input string and return user's input as
        well as extract a float from the input.

        Args:
            prompt (str): Prompt to request user input.

        Returns:
            tuple[str, float]: User's response, as well as an attempt
            to extract a float from the response.
        """
        response: str = input(input_prompt)
        return response, SqueezeFlowRheometer.find_num_in_str(response)

    @staticmethod
    def input_step_duration(default_duration: float = 300) -> float:
        """Gets duration of each step in seconds from user

        Args:
            default_duration (float, optional): default step duration in seconds. Defaults to 300

        Returns:
            float: duration of each step in seconds
        """
        dur_line, step_dur = SqueezeFlowRheometer.get_user_input(
            "Enter the duration of each step in seconds. Simply press Enter"
            f" for the default of {default_duration}s: "
        )
        if "config" in dur_line.lower() or len(dur_line) <= 0:
            step_dur = default_duration

        print(f"Step duration is {step_dur:.2f}s")
        return step_dur

    @staticmethod
    def input_retract_start_gap(sample_volume, settings) -> float:
        """Gets gap to approach to and retract from from the user for a retraction test.

        Returns:
            float: the target gap in mm
        """
        min_gap = (
            1000 * sample_volume / SqueezeFlowRheometer.HAMMER_AREA
        )  # mm, minimum gap before sample is squeeze beyond the plate
        while True:
            target_gap_line = input(
                "Enter the target gap to retract from in [mm]. If "
                "you want to use the gap in the settings file, just hit Enter: "
            )
            if "settings" in target_gap_line.lower() or len(target_gap_line) <= 0:
                target_gap = float(settings["retract_gap_mm"])
            else:
                target_gap = SqueezeFlowRheometer.find_num_in_str(target_gap_line)

            if target_gap < min_gap:
                print(
                    "That gap is too small! The sample will squeeze out past the "
                    f"edge of the plate. Try a gap larger than {min_gap:.2f}"
                )
            else:
                break
        print(f"Target gap is {target_gap:.2f}mm")
        return target_gap

    @staticmethod
    def input_retract_speed(settings: dict) -> float:
        """Gets retraction speed from the user for a retraction experiment.

        Returns:
            float: the target retraction speed in mm/s
        """
        target_speed_line = input(
            "Enter the retraction speed in [mm/s]. If you want to use the speed in the settings file, just hit Enter: "
        )

        # pylint: disable=unsubscriptable-object
        if "settings" in target_speed_line.lower() or len(target_speed_line) <= 0:
            target_speed = abs(float(settings["retract_speed_mms"]))
        else:
            target_speed = abs(SqueezeFlowRheometer.find_num_in_str(target_speed_line))
        print(f"Retraction speed is {target_speed:.1f}mm/s")
        return target_speed

    def load_settings(self):
        """Load device settings"""
        with open(self.settings_path, "r") as read_file:
            self.test_settings = json.load(read_file)

            self.K_P = self.test_settings["K_P"]
            self.K_I = self.test_settings["K_I"]
            self.K_D = self.test_settings["K_D"]
            self.decay_rate_r = self.test_settings["decay_rate_r"]
            self.a = self.test_settings["a"]
            self.b = self.test_settings["b"]
            self.c = self.test_settings["c"]
            self.d = self.test_settings["d"]
            self.ref_gap = self.test_settings["ref_gap"]
            self.default_duration = self.test_settings["test_duration"]

            self.variable_K_P = lambda er, tar: (self.a + self.b) / 2 + (
                self.a - self.b
            ) / 2 * math.tanh(self.c * ((er / tar) ** 2 - self.d))

    def create_data_file(self, file_heading: str):
        """Create a .csv data file for an SFR test

        Args:
            file_heading (str): first row of .csv file, a comma separated string of headers
        """
        file_path = os.path.join(self.data_folder, self.data_file_name)
        with open(file_path, "a") as datafile:
            datafile.write(file_heading)

    def get_figure_folder_path(self) -> str:
        """Get the path to the folder to save live-plotted figures from this day's tests."""
        self.figure_folder = os.path.join(
            self.data_folder, "Figures", self.get_day_date_str()
        )
        return self.figure_folder

    def create_figures_folder(self) -> str:
        """Create the folder to save live-plotteds figure from this day's test. Folder may already exist from a prior test on the same day, which is fine."""
        self.get_figure_folder_path()
        Path(self.figure_folder).mkdir(parents=True, exist_ok=True)
        return self.figure_folder

    def save_figure(self, fig: Figure):
        """Saves live-plotted figure

        Args:
            fig (Figure): Figure to be saved
        """
        self.figure_name = self.data_file_name.replace(
            "-data.csv", "-livePlottedFigure.png"
        )
        self.figure_path = os.path.join(self.figure_folder, self.figure_name)
        fig.show()
        # fig.draw()
        fig.savefig(self.figure_path, transparent=True)

    def end_test(self, fig: Figure = None):
        """Set the test as inactive, save the live-plotted figure, and then return the actuator to its home position.

        Args:
            fig (Figure, optional): Live-plotted figure to be saved. If not provided, it doesn't get saved
        """
        self.test_active = False

        try:
            self.save_figure(fig)
        except:
            print("Failed to save figure.")
        self.go_home_quiet_down()

    def get_day_date_str(self) -> str:
        """Gets a formatted date string down to the day

        Returns:
            str: yyyy-mm-dd formatted date string
        """
        return self.date.strftime("%Y-%m-%d")

    def get_second_date_str(self) -> str:
        """Gets a formatted date string down to the second. Hours are in 24-hour time

        Returns:
            str: yyyy-mm-dd_HH-MM-SS formatted date string
        """
        return self.date.strftime("%Y-%m-%d_%H-%M-%S")

    def write_data_to_file(self, output_params: list) -> str:
        """Write data to the data file

        Args:
            output_params (list): data ordered appropriately to line up with file headers

        Returns:
            str: line added to data file
        """

        file_path = os.path.join(self.data_folder, self.data_file_name)
        with open(file_path, "a") as datafile:
            dataline = ",".join(map(str, output_params)) + "\n"
            datafile.write(dataline)
            return dataline

    def get_perfect_slip_yield_stress(self) -> float:
        """Compute the yield stress assuming perfect slip and quasisteady. From Meeten (2000)

        Returns:
            float: yield stress assuming perfect slip and quasisteady
        """
        try:
            return (
                OpenScale.grams_to_N(self.force)
                * self.gap
                / self.visc_volume
                / math.sqrt(3)
            )
        except ZeroDivisionError:
            print("No usable volume for yield stress computation")
            return 0
        except:
            return 0

    def get_no_slip_yield_stress(self) -> float:
        """Compute the yield stress assuming no slip and quasisteady. From Scott (1935)

        Returns:
            float: yield stress assuming no slip and quasisteady
        """
        try:
            return (
                1.5
                * math.sqrt(math.pi)
                * OpenScale.grams_to_N(self.force)
                * (self.gap) ** 2.5
                / ((self.visc_volume) ** 1.5)
            )
        except ZeroDivisionError:
            print("No usable volume for yield stress computation")
            return 0
        except:
            return 0

    def get_gap(self, pos: float = None) -> float:
        """Computes the current gap in meters

        Args:
            pos (float, optional): Current position in mm. If not provided, will get from actuator

        Returns:
            float: gap between plates, measured in m
        """
        if pos is None:
            pos = self.steps_to_mm(self.get_pos())
        gap = (pos + self.start_gap) / 1000
        return gap

    def data_writing_thread_method(self, include_PID_values: bool = False):
        """Records data to csv

        Args:
            include_PID_values (bool, optional): Whether or not to include PID values like error
            and K_P in the data output. Only useful for PID-controlled tests. Defaults to False.
        """

        self.start_time = time()
        while True:
            cur_pos = self.get_pos()
            cur_pos_mm = self.steps_to_mm(cur_pos)
            tar_pos = self.get_variable_by_name("target_position")
            cur_vel = self.get_vel()
            cur_vel_mms = self.vel_to_mms(cur_vel)
            tar_vel = self.get_variable_by_name("target_velocity")
            max_speed = self.get_variable_by_name("max_speed")
            max_decel = self.get_variable_by_name("max_decel")
            max_accel = self.get_variable_by_name("max_accel")
            step_mode = self.get_variable_by_name("step_mode")
            vin_voltage = self.get_variable_by_name("vin_voltage")
            self.gap = self.get_gap(cur_pos_mm)  # set gap whether or not test is active

            # self.visc_volume=min(self.sample_volume,SqueezeFlowRheometer.HAMMER_AREA*self.gap)
            self.visc_volume = (
                self.sample_volume  # Carbopol keeps being predicted to over spread too soon
            )

            self.yield_stress_guess = self.get_perfect_slip_yield_stress()

            cur_time = time()
            cur_duration = cur_time - self.start_time

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
                self.force,
                self.target,
                self.start_gap / 1000.0,
                self.gap,
                self.eta_guess,
                self.yield_stress_guess,
                self.sample_volume,
                self.visc_volume,
                self.test_active,
                self.spread_beyond_hammer,
            ]

            # Only output these if the test requests it, otherwise they're
            # probably not in use and therefore meaningless
            if include_PID_values:
                output_params.extend(
                    [
                        self.error,
                        self.variable_K_P(self.error, self.target),
                        self.int_error,
                        self.K_I,
                        self.der_error,
                        self.K_D,
                    ]
                )

            self.write_data_to_file(output_params)

            self.times.append(cur_duration)
            self.forces.append(self.force)
            self.gaps.append(self.gap)
            self.yield_stress_guesses.append(self.yield_stress_guess)

            sleep(0.02)

            if (time() - self.start_time) >= SqueezeFlowRheometer.MAX_TEST_DURATION or (
                (not self.actuator_thread.is_alive()) and (time() - self.start_time) > 1
            ):
                print(f"Time since started: {(time() - self.start_time):.0f}")
                print(f"Actuator thread dead? {(not self.actuator_thread.is_alive())}")
                print("End of data-writing thread")
                break
        print("=" * 20 + " BACKGROUND IS DONE " + "=" * 20)

    def load_cell_thread_method(self, compute_errors: bool = False):
        """Continuously reads load cell and reports the upward force on the load cell

        Args:
            compute_errors (bool, optional): Whether to compute force error and integrated
            error / derivative error, which are used for PID force control. Defaults to False.
        """

        for _ in range(10):  # get rid of first few lines that aren't readings
            self.get_line()
        self.flush_old_lines()  # and get rid of any others that
        # were generated when we were busy setting up

        old_error = self.error

        cur_time = time()
        prev_time = cur_time

        while True:
            self.force = (
                self.wait_for_calibrated_measurement()
                * SqueezeFlowRheometer.FORCE_UP_SIGN
            )

            if compute_errors:
                prev_time = cur_time
                cur_time = time()
                dt_force = cur_time - prev_time

                old_error = self.error
                self.error = self.target - self.force
                self.int_error = self.int_error * math.exp(self.decay_rate_r * dt_force)
                self.int_error += (
                    ((old_error + self.error) / 2 * dt_force) if dt_force > 0 else 0
                )  # trapezoidal integration
                self.der_error = (
                    ((self.error - old_error) / dt_force) if dt_force > 0 else 0
                )  # first order backwards difference

            if (time() - self.start_time) >= SqueezeFlowRheometer.MAX_TEST_DURATION or (
                (not self.actuator_thread.is_alive())
                and (not self.data_writing_thread.is_alive())
                and (time() - self.start_time) > 1
            ):
                print("Stopping load cell reading")
                break
