#!/usr/bin/env python3
"""
Assorted utilities used by the other callback scripts
"""

import datetime
from math import pi

def deg2rad(x):
    """Convert an angle in degrees to radians"""
    return float(x) * pi / 180.0

def rad2deg(x):
    """Convert an angle in radians to degrees"""
    return float(x) * 180.0 / pi

def datestring_now():
    """Return a string of the form YYYYMMDDhhmmss"""
    now = datetime.datetime.now()
    return now.strftime('%Y%m%d%H%M%S')
