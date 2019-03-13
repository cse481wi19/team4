#! /usr/bin/env python

from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import Image
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
import pickle
from map_annotator.msg import PoseNames, UserAction

def transform_to_pose(matrix):
    pose = Pose()
    trans_vector = tft.translation_from_matrix(matrix)
    pose.position = Point(trans_vector[0], trans_vector[1], trans_vector[2])
    quartern = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quartern[0], quartern[1], quartern[2], quartern[3])
    return pose

def pose_to_transform(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix

class FaceDetector(object):
    def __init__(self):
      self.sub = rospy.Subscriber("/face_detector/people_tracker_measurements_array",PositionMeasurementArray, callback=self.callback)
      self.arr = PositionMeasurementArray()
      self.listener = tf.TransformListener()
      self.pose = None

    def callback(self, msg):
        self.arr = msg
        if len(self.arr.people) is not 0:
          try:
            (position, quaternion) = self.listener.lookupTransform('base_link', 'head_camera_rgb_optical_frame', rospy.Time(0))
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            facePose = Pose()
            facePose.position = self.arr.people[0].pos

            # newMat = np.dot(pose_to_transform(pose), pose_to_transform(facePose))
            ps = PoseStamped()
            # ps.pose = transform_to_pose(newMat)

            ps.pose.position.x = facePose.position.z
            ps.pose.position.y = -facePose.position.x
            ps.pose.position.z = -facePose.position.y
            ps.pose.position.x += pose.position.x
            ps.pose.position.y += pose.position.y
            ps.pose.position.z += pose.position.z
            ps.pose.position.x -= 0.45
            ps.pose.position.y -= 0
            ps.pose.position.z -= 0.43

            ps.pose.orientation.x = 0.727
            ps.pose.orientation.y = 0
            ps.pose.orientation.z = 0
            ps.pose.orientation.w = 0.687
            ps.header.frame_id = 'base_link'
            self.pose = ps
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass