#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import math
import rospy

LOOK_AT_ACTION_NAME = 'head_controller/point_head.'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = None  # TODO: Minimum pan angle, in radians.
    MAX_PAN = None  # TODO: Maximum pan angle, in radians.
    MIN_TILT = None  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = None  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self.joint_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.head_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        # TODO: Wait for both servers
        pass

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        # TODO: Wait for result
        rospy.logerr('Not implemented.')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result

        rospy.logerr('Not implemented.')