"""Reads force from load cell and plots to a figure with a
rolling time-average to check load cell calibration"""

from time import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import openscale

scale = openscale.OpenScale()

readings = []
times = []
means = []
MAX_TIME_WINDOW = 7  # how many seconds of readings to store and plot.
start_time = time()

# style.use("fivethirtyeight")
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

COLOR1 = "C0"
COLOR2 = "C1"


def animate():
    """Plot load cell readings."""
    # global ax1, MAX_TIME_WINDOW
    ax1.clear()

    # Store data & timestamps locally to prevent race conditions due to multithreading
    times_temp = times[:]
    readings_temp = readings[:]
    means_temp = means[:]
    if len(times_temp) < 1:
        return

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Force [g]", color=COLOR1)
    ax1.plot(times_temp, readings_temp, COLOR1, label="data")
    ax1.plot(times_temp, means_temp, COLOR2, label="time average")
    plt.xlim(min(times_temp), max(times_temp, MAX_TIME_WINDOW))
    plt.grid()


def get_data():
    """Read load cell, average data, and truncate away old readings"""
    # global readings, times, MAX_TIME_WINDOW  # , scale
    while True:
        weight = scale.wait_for_calibrated_measurement()

        # print("{:6.2f}{:}".format(weight, scale.units))
        print(f"{weight:6.2f}{scale.units:}")

        # Grab new data & timestamp
        readings.append(weight)
        times.append(time() - start_time)
        means.append(sum(readings) / len(readings))

        # Throw away data & timestamps that are too old. There is definitely a smarter/faster way to do this.
        while times[-1] - times[0] > MAX_TIME_WINDOW:
            readings.pop(0)
            times.pop(0)
            means.pop(0)


gd = threading.Thread(name="get_data", target=get_data)

gd.start()

ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
