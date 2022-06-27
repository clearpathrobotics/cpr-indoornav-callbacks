#!/usr/bin/env python3
"""
Generic callbacks for starting/stopping video recording & saving still images from a camera image topic
"""

import rospy
import actionlib

from cpr_indoornav_callbacks.utils import *
from video_recorder_msgs.msg import SaveImageAction, SaveImageGoal
from video_recorder_msgs.msg import StartRecordingAction, StartRecordingGoal
from video_recorder_msgs.msg import StopRecordingAction, StopRecordingGoal

## Saves a single image from an image topic
class SaveImageCallback:
    ## Create a callback instance to save a single image from the camera
    #
    #  \param img_action   The ROS action to call to capture an image from the camera
    #  \param delay        An optional delay to apply before taking the picture
    #  \param filename     An optional filename to pass to the action
    def __init__(self,
                 img_action='/camera/image_raw/save_image',
                 delay=0,
                 filename=''):

        rospy.init_node('image_callback_node', anonymous=True)

        self.img_action_name = img_action
        self.delay = delay
        self.filename = filename

    ## Call the action to capture a frame
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.img_action_name, SaveImageAction)
            client.wait_for_server()

            goal = SaveImageGoal()
            goal.delay = self.delay
            goal.filename = self.filename

            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()

            rospy.loginfo('Saved image to {0}'.format(result.path))

        except Exception as e:
            ok = False
            rospy.logerror("Failed to run PTZ callback: {0}".format(e))

        finally:
            return ok

## Start recording video data from a camera
class StartRecordingCallback:
    ## Create a callback instance to start recording from the camera
    #
    #  \param rec_action     The ROS action to call to start recording
    #  \param duration       The desired duration to record in seconds (if 0 or negative we will record forever)
    #  \param filename       Optional filename for the resulting AVI file.
    def __init__(self,
                 rec_action="/camera/image_raw/start_recording",
                 duration=0,
                 filename=""):

        rospy.init_node('start_video_callback_node', anonymous=True)

        # max duration is an unsigned int64, so if it's negative treat it as zero
        # always force it to be an int
        if max_duration < 0:
            max_duration = 0
        max_duration = int(max_duration)

        self.filename = filename
        self.duration = duration
        self.rec_action_name = rec_action


    ## Call the action to start recording video
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.rec_action_name, StartRecordingAction)
            client.wait_for_server()

            goal = StartRecordingGoal()
            goal.duration = self.duration
            goal.filename = self.filename

            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()

            rospy.loginfo('Saved video to {0}'.format(result.path))


        except Exception as e:
            ok = False
            rospy.logerror("Failed to start recording video: {0}".format(e))

        finally:
            return ok


## Stop recording video data from a camera
class StopRecordingCallback:
    ## Create a callback instance to stop recording from the camera
    #
    #  \param rec_action    The ROS action to call to stop recording
    def __init__(self,
                 rec_action="/camera/image_raw/stop_recording"):

        rospy.init_node('stop_video_callback_node', anonymous=True)
        self.rec_action_name = rec_action

    ## Call the action to stop recording video
    #
    #  \return True if the action was called successfully, otherwise False
    def run(self):
        ok = True
        try:
            client = actionlib.SimpleActionClient(self.rec_action_name, StopRecordingAction)
            client.wait_for_server()

            goal = StopRecordingGoal()

            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()

            rospy.loginfo('Recording stopped {0} {1}s'.format(result.path, result.duration))

        except Exception as e:
            ok = False
            rospy.logerror("Failed to stop recording video: {0}".format(e))

        finally:
            return ok
