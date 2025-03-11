"""Tares load cell"""

from openscale import OpenScale

scale = OpenScale()
scale.tare(wait_time=10, n=2000)
