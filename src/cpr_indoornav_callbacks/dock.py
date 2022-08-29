#!/usr/bin/env python3
"""
Generic callbacks for docking and undocking from a wireless charger
"""

import actionlib
import dynamic_reconfigure.client
import json
import math
import rospy
import urllib.request

from cpr_indoornav_callbacks.utils import *
from cpr_gps_navigation_msgs.msg import DockGoal, UITaskAction, UITaskGoal
from geometry_msgs.msg import Twist

ENABLE_PRE_DOCK=1
DISABLE_PRE_DOCK=0

## Docks the robot with its charger
class DockCallback:
    ## Create a callback instance to dock the robot
    def __init__(self, action='/charge_robot', backpack_host="10.252.252.1", backpack_port=5000):
        self.backpack_host = backpack_host
        self.backpack_port = backpack_port
        self.action = action

    ## Get the location and orientation of the charging dock
    #  This location is sent to the docking action server as a dynamic_reconfigure serice call
    def getDockLocation(self):
        location = None
        try:
            docks = []
            with urllib.request.urlopen('http://{0}:{1}/api/v2/maps/markers'.format(self.backpack_host, self.backpack_port)) as url:
                data = json.loads(url.read().decode())
                features = data['features']
                for f in features:
                    if f['properties']['item_type'] == 'RZR_CHARGE_DOCK':
                        docks.append(f)

            if len(docks) == 0:
                rospy.logerr("No docks were found on the map. Please add one")
            elif len(docks) > 1:
                rospy.logwarn("Multiple docks were found on the map. Using the first one")
                # TODO: use tf to look up the robot's location and dynamically choose the closest dock
                location = Twist()
                location.angular.z = docks[0]['properties']['yaw']
                location.linear.x = docks[0]['geometry']['coordinates'][0] + 0.25 * math.cos(location.angular.z)
                location.linear.y = docks[0]['geometry']['coordinates'][1] + 0.25 * math.sin(location.angular.z)

            else:
                location = Twist()
                location.angular.z = docks[0]['properties']['yaw']
                location.linear.x = docks[0]['geometry']['coordinates'][0] + 0.25 * math.cos(location.angular.z)
                location.linear.y = docks[0]['geometry']['coordinates'][1] + 0.25 * math.sin(location.angular.z)

        except Exception as err:
            rospy.logerr("Failed to locate the dock on the map: {0}".format(err))
        finally:
            return location


    ## Uses dynamic_reconfigure to send the dock location to the target-tracker
    #
    #  \param location  a Twist object with the location of the dock in the map frame
    def configureDockLocation(self, location):
        client = dynamic_reconfigure.client.Client('/target_tracker_client_node')
        params = {
            'target_x': location.linear.x,
            'target_y': location.linear.y,
            'target_yaw': location.angular.z,
            'dock_with_outdoornav': False
        }
        client.update_configuration(params)

    ## Call the action to dock the robot
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            # Before we can actually dock we need to look up the dock's location on the map and send it
            # to the target-tracker
            dockLocation = self.getDockLocation()
            if dockLocation is None:
                raise Exception("Cannot dock if there is no dock on the map")
            self.configureDockLocation(dockLocation)

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
            rospy.logerr("Failed to run dock action: {0}".format(e))

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
            rospy.logerr("Failed to run undock action: {0}".format(e))

        finally:
            return ok
