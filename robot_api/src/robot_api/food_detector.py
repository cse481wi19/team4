#! /usr/bin/env python

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
import robot_api
import math
import pickle
from perception_msgs.msg import ObjectPosition

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

class FoodDetector(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/food_pub", ObjectPosition, callback=self.callback)
        self.pub = rospy.Publisher("/feed_pub", Pose, queue_size=5)
        self.pose = None

    def callback(self, msg):
        pose = Pose()
        pose.position.x = msg.values[0]
        pose.position.y = msg.values[1]
        pose.position.z = msg.values[2]

        ps = PoseStamped()

        ps.pose.position.x = pose.position.x
        ps.pose.position.y = pose.position.y
        ps.pose.position.z = pose.position.z
        # ps.pose.position.x -= 0.35
        ps.pose.position.x -= 0.05
        # ps.pose.position.z -= 0.15

        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = 0
        ps.pose.orientation.w = 0
        ps.header.frame_id = 'base_link'
        self.pose = ps
        pass


    def getFood(self):
        pose = Pose()
        pose.position.x = 1
        self.pub.publish(pose)

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass

def main():
    rospy.init_node('pbd')
    wait_for_time()
    fooddetector = robot_api.FoodDetector()
    rospy.sleep(2)
    pose = Pose()
    pose.position.x = 1
    fooddetector.pub.publish(pose)


if __name__ == '__main__':
    main()