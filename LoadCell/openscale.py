"""Provides OpenScale class, wrapper for interfacing with OpenScale board"""

from time import time
import json
import re
import math
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
import os.path


class OpenScale:
    """Wrapper for interfacing with OpenScale board and getting calibrated
    force readings in coherent units from the load cell"""

    OLD_READING_KEEP_AMOUNT = 10
    """How many old readings to keep"""
    OUTLIER_QUORUM_AMOUNT = 5
    """How many points must be within the outlier jump threshold of the new measurement for it to be considered not an outlier."""
    OUTLIER_JUMP_THRESHOLD = 10
    """The maximum acceptable jump in grams between two force readings"""

    def __init__(self):
        try:
            self.ser = serial.Serial(self.get_COM_port(), 115200)
        except serial.SerialException():
            print(
                "Could not open port to read load cell. Is the "
                "OpenScale board plugged in to the computer?"
            )

        self.outlier_threshold = (
            100  # g, if a measurement is beyond this limit, throw it out
        )
        self.old_readings = [None] * (
            OpenScale.OLD_READING_KEEP_AMOUNT + 1
        )  # also have to store current value

        self.config_path = os.path.join("LoadCell", "config.json")
        """Location of load cell config file"""
        self.config: dict
        """Dict of configuration and calibration data for load cell"""
        self.tare_value: float
        """Tare constant for the load cell"""
        self.calibration: float
        """Calibration constant for the load cell"""
        self.units: str
        """Units the load cell is calibrated to report force in"""
        self.outlier_threshold: float
        self.force_limit: float
        """Max allowable force before the rheometer automatically ends the test."""
        self.load_config()

    def load_config(self) -> dict:
        """Load load cell calibration and configuration info from file"""
        try:
            with open(self.config_path, "r") as read_file:
                self.config = json.load(read_file)
                if "tare" in self.config:
                    self.tare_value = self.config["tare"]
                if "calibration" in self.config:
                    self.calibration = self.config["calibration"]
                if "units" in self.config:
                    self.units = self.config["units"]
                if "max_force" in self.config:
                    self.outlier_threshold = self.config["max_force"]
                    if "limit_fraction" in self.config:
                        self.force_limit = (
                            self.config["max_force"] * self.config["limit_fraction"]
                        )
        except:
            self.config = {}
        return self.config

    def flush_old_lines(self):
        """Clears existing serial buffer"""
        self.ser.reset_input_buffer()

    def get_line(self) -> bytes:
        """Grabs the next line of serial input from the OpenScale

        Returns:
            bytes: next line from OpenScale
        """
        return self.ser.readline()

    @staticmethod
    def ser_to_reading(serial_line: bytes) -> int:
        """Takes in serial line and returns raw reading reported therein

        Args:
                serial_line (bytes): a line of load cell serial input

        Returns:
                int: the load cell reading in that line
        """
        try:
            num_string = serial_line.decode("utf-8")[:-3]  # just get the actual content
            reading = int(num_string)
            return reading
        except:
            print(serial_line)
            return None

    def reading_to_units(self, reading: int) -> float:
        """Takes in raw load cell reading and returns calibrated measurement

        Args:
                reading (int): raw value from load cell input

        Raises:
            Exception: If load cell has not been calibrated, cannot give a calibrated measurement.

        Returns:
                float: calibrated measurement in units the load cell is calibrated to
        """

        # pylint: disable=broad-exception-raised
        if (
            ("tare" not in self.config)
            or ("calibration" not in self.config)
            or ("units" not in self.config)
        ):
            raise Exception(
                "Load cell has not been calibrated, cannot report a calibrated measurement."
            )
        if reading is None:
            return None
        return (reading - self.tare_value) / self.calibration

    def get_reading(self) -> int:
        """Grabs the next serial line and returns the reading

        Returns:
            int: raw reading from OpenScale
        """
        return OpenScale.ser_to_reading(self.get_line())

    def wait_for_reading(self) -> int:
        """Wait aas long as needed to get next load cell reading."""
        reading = None
        while reading is None:
            reading = self.get_reading()

        if (
            ("tare" in self.config)
            and ("calibration" in self.config)
            and ("units" in self.config)
        ):
            self.old_readings.pop(0)
            self.old_readings.append(self.reading_to_units(reading))
        return reading

    def get_calibrated_measurement(self) -> float:
        """Grabs the next serial line and returns the calibrated measurement

        Returns:
            float: force measurement in units chosen during calibration
        """
        meas = self.reading_to_units(self.get_reading())
        self.old_readings.pop(0)
        self.old_readings.append(meas)
        return meas

    def wait_for_calibrated_measurement(self) -> float:
        """Waits for the next valid reading and returns the calibrated measurement

        Returns:
            float: force measurement in units chosen during calibration
        """
        meas = None
        while meas is None:
            meas = self.reading_to_units(self.wait_for_reading())
            if self.check_if_outlier(
                meas
            ):  # if it's too far from all of the previous readings
                meas = None
        return meas

    def check_if_outlier(self, measurement: float) -> bool:
        """Checks if a measurement is too far from enough previous stored values.
        If it is within a certain threshold for at least the quorum amount of values, returns false. Otherwise returns true.

        Args:
            measurement (float): the new measurement to check if it's an outlier

        Returns:
            bool: True if it's an utlier (too far from prior measurements),
                False if it's considered a real measurement.
        """

        distance_from_old_measurement: list[float] = [
            abs(measurement - (old if old is not None else 0))
            for old in self.old_readings[0:-1]
        ]
        close_enough: list[bool] = [
            d <= OpenScale.OUTLIER_JUMP_THRESHOLD for d in distance_from_old_measurement
        ]
        number_close_enough: int = sum(close_enough)
        is_outlier = number_close_enough < OpenScale.OUTLIER_QUORUM_AMOUNT
        return is_outlier

    @staticmethod
    def grams_to_N(f: float) -> float:
        """Takes in force in grams and converts to Newtons

        Args:
                f (float): force in grams

        Returns:
                float: force in Newtons
        """
        return 0.00980665 * f

    def tare(self, wait_time: int = 120, n: int = 1000) -> float:
        """Performs taring of the load cell. Saves tare value

        Args:
            wait_time (int, optional): Time to wait for load cell creep to occur. Defaults to 120.
            n (int, optional): Number of samples to average over. Defaults to 1000.

        Returns:
            float: tare value - the average reading when the load cell has no force applied
        """

        print(
            f"Taking first {wait_time:d} seconds to let load cell creep happen. "
            "This will lead to a more accurate tare value."
        )
        start_time = time()
        while time() - start_time <= wait_time:
            remaining = wait_time - (time() - start_time)
            print(f"{remaining:5.1f}: {self.get_line()}")
        self.flush_old_lines()  # and clear any extra lines that may
        # have been generated, we don't need them

        readings = [0] * n
        print("Now recording values for taring")
        for i in range(n):
            readings[i] = self.wait_for_reading()
            print(f"{i:5d}: {readings[i]:8d}")

        # Throw out top 1% and bottom 1%
        remove_rate = 0.01
        readings = sorted(readings)
        readings = readings[
            math.floor(remove_rate * n) : math.floor((1 - remove_rate) * n)
        ]
        print(
            f"Keeping the middle {(1 - 2 * remove_rate):.0%} of samples "
            "to remove outliers due to noise"
        )

        tare_value = sum(readings) / len(readings)
        print(f"The tare value is {tare_value:.2f}")

        reading_std = np.std(readings)
        max_reading = max(readings)
        min_reading = min(readings)
        print(f"min: {min_reading}, max: {max_reading}, standard dev: {reading_std}")
        readings_over = sorted(i for i in readings if i >= tare_value + reading_std)
        readings_under = sorted(
            (i for i in readings if i <= tare_value - reading_std), reverse=True
        )
        print(
            f"Number over 1std: {len(readings_over)}, Number under 1std: {len(readings_under)}, "
            f"total outside 1std: {(len(readings_over) + len(readings_under))}"
        )
        print(
            f"Number within 1std: {(len(readings) - len(readings_over) - len(readings_under))}"
        )
        plt.hist(sorted(np.array(readings) - tare_value), 50)
        plt.xlabel("Deviation from the mean")
        plt.ylabel("Number of samples")
        plt.title(f"Middle {(1 - 2 * remove_rate):.0%} of readings")
        plt.show()

        self.config["tare"] = tare_value
        if "limit_fraction" not in self.config:
            self.config["limit_fraction"] = 0.8
        with open(self.config_path, "w") as write_file:
            json.dump(self.config, write_file)

        self.tare_value = tare_value
        return tare_value

    def calibrate(
        self, tare_first: bool = False, n: int = 1000, report_duration: int = 10
    ) -> float:
        """Performs calibration of load cell

        Args:
            n (int, optional): Number of samples to average over. Defaults to 1000.
            report_duration (int, optional): Amount of time to report values after
            calibration is complete. Defaults to 10.

        Returns:
            float: _description_
        """
        if "tare" not in self.config:
            print("Load cell has not been tared, will now perform taring.")
            tare_first = True
        if tare_first:
            input(
                "Please remove any weights you had placed. Press enter to being taring process."
            )
            self.tare(n=n)
            print("Taring complete. Now to calibrate.")

        total = 0

        # Have the user place the calibration weight and ask what the weight is
        print("Please place the calibration weight(s).")
        cal_weight_str = input(
            "Enter the total calibration weight with units (ex: 50g): "
        )

        cal_weight_str = cal_weight_str.replace(" ", "")  # filter out spaces

        # separate weight from units
        temp = re.compile("([0-9.]+)([a-zA-Z]+)")
        res = temp.match(cal_weight_str).groups()
        cal_weight = abs(float(res[0]))
        self.units = res[1]
        self.config["units"] = self.units

        # Also set max force limit while you're at it
        max_force_str = input(f"Enter the load cell capacity in {self.units}: ")
        self.outlier_threshold = abs(float(max_force_str))
        self.config["max_force"] = self.outlier_threshold
        self.force_limit = self.outlier_threshold * self.config["limit_fraction"]

        for i in range(10):  # ignore the first few lines, they're not data
            self.get_line()
        self.flush_old_lines()  # and clear any extra lines that may have been
        # generated, we don't need them

        readings = [0] * n
        for i in range(n):
            # reading = self.get_reading()
            reading = self.wait_for_reading() - self.tare_value
            readings[i] = reading
            print(f"{i:5d}: {reading:10.1f}")
            total += reading - self.tare_value

        # Throw out top 1% and bottom 1%
        remove_rate = 0.01
        readings = sorted(readings)
        readings = readings[
            math.floor(remove_rate * n) : math.floor((1 - remove_rate) * n)
        ]
        print(
            f"Keeping the middle {(1 - 2 * remove_rate):.0%} of "
            "samples to remove outliers due to noise"
        )

        # average = total / N
        average = sum(readings) / len(readings)
        print(f"The calibration average is {average:.2f}")

        reading_std = np.std(readings)
        max_reading = max(readings)
        min_reading = min(readings)
        print(f"min: {min_reading}, max: {max_reading}, standard dev: {reading_std}")
        readings_over = sorted(i for i in readings if i >= average + reading_std)
        readings_under = sorted(
            (i for i in readings if i <= average - reading_std), reverse=True
        )
        print(
            f"Number over 1std: {len(readings_over)}, Number under 1std: {len(readings_under)}, "
            f"total outside 1std: {(len(readings_over) + len(readings_under))}"
        )
        print(
            f"Number within 1std: {(len(readings) - len(readings_over) - len(readings_under))}"
        )
        plt.hist(sorted(np.array(readings) - average), 50)
        plt.xlabel("Deviation from the mean")
        plt.ylabel("Number of samples")
        plt.title(f"Middle {(1 - 2 * remove_rate):.0%} of readings")
        plt.show()

        calibration = -average / cal_weight
        self.config["calibration"] = calibration
        with open(self.config_path, "w") as write_file:
            json.dump(self.config, write_file)

        print(f"The calibration value is {calibration:.2f}")
        input(
            "You should now change the weights. For the next 10 seconds, "
            "the measured weight will be constantly printed. Press enter to begin."
        )
        self.flush_old_lines()

        start_time = time()
        while (time() - start_time) <= report_duration:
            # pylint: disable=invalid-unary-operand-type
            weight = -self.wait_for_calibrated_measurement()
            print(f"{weight:6.2f}{self.units}")

        return calibration

    def check_tare(self):
        """Check if load cell is within tare, otherwise tare it."""
        weight = self.wait_for_calibrated_measurement()
        if abs(weight) > 0.5:
            ans = input(
                f"The load cell is out of tare! Current reading is {weight:.2f}{self.units}. "
                "Do you want to tare it now? (y/n) "
            )
            if ans == "y":
                self.tare()

    def get_COM_port(self):
        """Finds COM ports connected to USB

        Returns:
            _type_: _description_
        """
        com_ports = serial.tools.list_ports.comports()
        usable_port = ""
        for port in com_ports:
            if port.vid:  # if
                usable_port = port.device
        return usable_port
