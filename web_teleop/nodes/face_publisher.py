#!/usr/bin/env python


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

class Face(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/face_detector/people_tracker_measurements_array",PositionMeasurementArray, callback=self.callback)
        self.pub = rospy.Publisher("face_coord", Quaternion, queue_size=5)
        self.q = None

    def callback(self, msg):
        self.arr = msg
        if len(self.arr.people) is not 0:
            self.q = Quaternion()
            self.q.x = msg.people[0].covariance[0]
            self.q.y = msg.people[0].covariance[1]
            self.q.z = msg.people[0].covariance[2]
            self.q.w = msg.people[0].covariance[3]
            self.pub.publish(self.q)

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass

def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    faces = Face()
    rospy.spin()


if __name__ == '__main__':
    main()


