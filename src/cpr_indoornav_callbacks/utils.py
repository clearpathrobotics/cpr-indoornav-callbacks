#!/usr/bin/env python3

import datetime

def datestring_now():
    """Return a string of the form YYYYMMDDhhmmss"""
    now = datetime.datetime.now()
    return now.strftime('%Y%m%d%H%M%S')
