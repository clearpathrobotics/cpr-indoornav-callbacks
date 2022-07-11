#!/usr/bin/env python

#!/usr/bin/env python3
"""
Generic callbacks for starting/stopping video recording & saving still images from a camera image topic
"""

import rospy
import actionlib

from cpr_indoornav_callbacks.utils import *
from cpr_gps_navigation_msgs.msg import DockGoal, UITaskAction, UITaskGoal

ENABLE_PRE_DOCK=1
DISABLE_PRE_DOCK=0

## Docks the robot with its charger
class DockCallback:
    ## Create a callback instance to dock the robot
    def __init__(self, action='/charge_robot'):

        self.action = action

    ## Call the action to dock the robot
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.action, UITaskAction)
            client.wait_for_server()

            goal = UITaskGoal()
            goal.floats = [DockGoal.DOCK, DISABLE_PRE_DOCK]

            client.send_goal(goal)
            rospy.loginfo('Dock action started')
            client.wait_for_result()
            result = client.get_result()
            rospy.loginfo('Dock action finished: {0}'.format(result))


        except Exception as e:
            ok = False
            rospy.logerror("Failed to run dock action: {0}".format(e))

        finally:
            return ok

## Undocks the robot from its charger
class UndockCallback:
    ## Create a callback instance to undock the robot
    def __init__(self, action='/charge_robot'):

        self.action = action

    ## Call the action to dock the robot
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.action, UITaskAction)
            client.wait_for_server()

            goal = UITaskGoal()
            goal.floats = [DockGoal.UNDOCK, DISABLE_PRE_DOCK]

            client.send_goal(goal)
            rospy.loginfo('Undock action started')
            client.wait_for_result()
            result = client.get_result()
            rospy.loginfo('Undock action finished: {0}'.format(result))


        except Exception as e:
            ok = False
            rospy.logerror("Failed to run undock action: {0}".format(e))

        finally:
            return ok
