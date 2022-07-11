#!/usr/bin/env python3

import rospy
import actionlib

from math import pi
from ptz_control_msgs.msg import PtzAction, PtzGoal

## Generic callback for PTZ camera endpoints.
#
#  The caller must assign the PTZ values, e.g. from a rosparam, or other interface
#
#  This class does not actually capture the image; use cpr_indoornav_callbacks.camera.ImageCallback for that functionality
class PtzCallback:
    ## Create a callback instance to move to the PTZ position
    #
    #  \param pan          The requested pan position in radians
    #  \param tilt         The requested tilt position in radians
    #  \param zoom         The requested zoom value. Units and range are camera-specific
    #  \param ptz_action   The ROS action to call to move the camera
    def __init__(self, pan, tilt, zoom, ptz_action="/ptz/move_ptz"):

        self.ptz_action_name = ptz_action
        self.pan = pan
        self.tilt = tilt
        self.zoom = zoom

    ## Call the action to move the PTZ camera to the desired location
    #
    #  \return True if the action was handled successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.ptz_action_name, PtzAction)
            rospy.loginfo('Waiting for {0} to come up...'.format(self.ptz_action_name))
            client.wait_for_server()

            pan = self.pan
            tilt = self.tilt
            zoom = self.zoom

            rospy.loginfo("Requesting PTZ move to position {0} {1} {2}".format(pan, tilt, zoom))
            goal = PtzGoal()
            goal.pan = float(pan)
            goal.tilt = float(tilt)
            goal.zoom = float(zoom)
            client.send_goal(goal)

            rospy.loginfo('Waiting for result...')
            client.wait_for_result()

            result = client.get_result()
            ok = result.success
            rospy.loginfo(result)

        except Exception as e:
            ok = False
            rospy.logerr("Failed to run PTZ callback: {0}".format(e))

        finally:
            return ok
