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
import pickle
from map_annotator.msg import PoseNames, UserAction

class ArTagReader(object):
    def __init__(self):
        self.markers = []
        self.savedMarkers = []

    def callback(self, msg):
        self.markers = msg.markers

    def update(self):
        self.savedMarkers = self.markers

    def getTag(self, tag):
        for marker in self.savedMarkers:
            if marker.id == int(tag):
                result = PoseStamped()
                result.pose = marker.pose
                result.header = marker.header
                return result

class Manager(object):
    def __init__(self, database, arm, gripper, facedetector, fooddetector):
        self.db = database
        self.db.load()
        self.arm = arm
        self.gripper = gripper
        self.reader = ArTagReader()
        self.sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self.reader.callback)
        self.listener = tf.TransformListener()
        self.facedetector = facedetector
        self.fooddetector = fooddetector
        self.savedBefore = False

    # Currently only add pose relative to tag instead of base_link
    def add(self, name, tag):
        # Calculate the coordinate of wrist based on base_link frame
        if name not in self.db.list():
            self.reader.update()
            self.savedBefore = True
            self.fooddetector.getFood()
            rospy.sleep(3)
        (position, quaternion) = self.listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        # If the relative frame is base_link, dont need to compute new coordinate
        if tag == 'base_link':
            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = 'base_link'
            self.db.add(name, ps, tag)
        elif tag == 'open' or tag == 'close' or tag == 'face':
        	self.db.add(name, None, tag)
        elif tag == 'food':
            tagPose = self.fooddetector.pose
            if tagPose == None:
                print("Frame does not exist")
            else:
                t_pos = tagPose.pose
                t_mat = pose_to_transform(t_pos)
                w_mat = pose_to_transform(pose)
                offset = np.dot(np.linalg.inv(t_mat), w_mat)
                self.db.add(name, offset, tag)
        else:
            # Get the coordinate of tag and compute the offset between tag and wrist
            tagPose = self.reader.getTag(tag)
            if tagPose == None:
                print("Frame does not exist")
            else:
                t_pos = tagPose.pose.pose
                t_mat = pose_to_transform(t_pos)
                w_mat = pose_to_transform(pose)
                offset = np.dot(np.linalg.inv(t_mat), w_mat)
                self.db.add(name, offset, tag)


    def run(self, name):
        self.reader.update()
        self.savedBefore = False
        self.fooddetector.getFood()
        rospy.sleep(3)
        if self.db.get(name) is not None:
            for offset, tag in self.db.get(name):
                if tag == 'base_link':
                    self.arm.move_to_pose(offset)
                elif tag == 'open':
                    self.gripper.open()
                elif tag == 'close':
                    self.gripper.close()
                elif tag == 'face':
                    if self.facedetector.pose is not None:
                        self.arm.move_to_pose(self.facedetector.pose)
                    else:
                        print("No face is detected.")
                elif tag == 'food':
                    tagPs = self.fooddetector.pose
                    if tagPs == None:
                        print("Frame does not exist")
                    else:
                        # Get the current transform of the given tag
                        tagMat = pose_to_transform(tagPs.pose)
                        # Compute the new coordinate based on pre-set frame
                        newTrans = np.dot(tagMat,offset)
                        pose = transform_to_pose(newTrans)
                        result = PoseStamped()
                        result.pose = pose
                        result.header = tagPs.header
                        # Move to the new coordinate
                        self.arm.move_to_pose(result)
                    # if self.fooddetector.pose is not None:
                    #     self.arm.move_to_pose(self.fooddetector.pose)
                    # else:
                    #     print("No food is detected.")
                else:
                    tagPs = self.reader.getTag(tag)
                    if tagPs == None:
                        print("Frame does not exist")
                    else:
                        # Get the current transform of the given tag
                        tagMat = pose_to_transform(tagPs.pose.pose)
                        # Compute the new coordinate based on pre-set frame
                        newTrans = np.dot(tagMat,offset)
                        pose = transform_to_pose(newTrans)
                        result = PoseStamped()
                        result.pose = pose
                        result.header = tagPs.header
                        # Move to the new coordinate
                        self.arm.move_to_pose(result)
        else:
            print("Pose does not exist")

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