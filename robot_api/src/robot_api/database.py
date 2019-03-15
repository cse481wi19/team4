#!/usr/bin/env python

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
import rospy
import actionlib
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import tf.transformations as tft
import robot_api
import math
import os
import pickle
from map_annotator.msg import PoseNames, UserAction

class Database(object):
    def __init__(self):
        self._markers = {}
        self.db_path = os.path.join('/home/team4/', "marker_db.p")

    def get(self, name):
        if name in self._markers:
            return self._markers[name]
        else:
            return None


    def add(self, name, offset, tag):
        if name not in self._markers:
            self._markers[name] = []
        self._markers[name].append((offset, tag))


    def delete(self, name):
        if name in self._markers:
            del self._markers[name]

    def list(self):
        return self._markers.keys()

    def load(self):
        try:
            with open(self.db_path, 'r') as f:
                self._markers = pickle.load(f)
        except IOError as e:
            rospy.logwarn('No storage information: {}'.format(e))


    def save(self):
        try:
            with open(self.db_path, 'w') as f:
                pickle.dump(self._markers, f)
        except IOError as e:
            rospy.logwarn('No storage information: {}'.format(e))