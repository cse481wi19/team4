#!/usr/bin/env python

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from web_teleop.srv import CommandService, CommandServiceResponse
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
import rospy
import actionlib
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import tf.transformations as tft
import robot_api
import math
import sys
import pickle
from map_annotator.msg import PoseNames, UserAction

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass

class CommandServer(object):
    def __init__(self):
        self.gripper = robot_api.Gripper()
        self.arm = robot_api.Arm()
        self.database = robot_api.Database()
        self.facedetector = robot_api.FaceDetector()
        self.fooddetector = robot_api.FoodDetector()
        self.controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        self.server = robot_api.Manager(self.database, self.arm, self.gripper, self.facedetector, self.fooddetector)

    def parse_command(self, request):
        rospy.loginfo(request.command)
        sys.stderr.write(request.command)
        if request.command == "run":
            self.server.run(request.args[0])
            sys.stderr.write(request.command)
            print(request.args[0])
            return CommandServiceResponse()

        if request.command == "face":
            if self.facedetector.pose is not None:
                self.arm.move_to_pose(self.facedetector.pose)
            else:
                self.gripper.open()
            return CommandServiceResponse()
        if request.command == "reset":
            ps = PoseStamped()
            ps.pose.position.x = 0.46
            ps.pose.position.y = -0.43
            ps.pose.position.z = 1.05
            ps.pose.orientation.x = 0.38
            ps.pose.orientation.y = 0.08
            ps.pose.orientation.z = 0.23
            ps.pose.orientation.w = 0.89
            ps.header.frame_id = 'base_link'
            self.arm.move_to_pose(ps)
        elif request.command == "relax":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            self.controller_client.send_goal(goal)
            self.controller_client.wait_for_result(rospy.Duration(5))
            return CommandServiceResponse()

        elif request.command == "move":
            ps = PoseStamped()
            ps.pose.position.x = float(request.args[0])
            ps.pose.position.y = float(request.args[1])
            ps.pose.position.z = float(request.args[2])
            ps.header.frame_id = 'base_link'
            self.arm.move_to_pose(ps)
            return CommandServiceResponse()

        elif request.command == "close":
            self.gripper.close()
            return CommandServiceResponse()

        elif request.command == "open":
            self.gripper.open()
            return CommandServiceResponse()

        elif request.command == "stop":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            self.controller_client.send_goal(goal)
            self.controller_client.wait_for_result(rospy.Duration(1))
            return CommandServiceResponse()

        elif request.command == "restart":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.RUNNING
            goal.updates.append(state)
            self.controller_client.send_goal(goal)
            self.controller_client.wait_for_result(rospy.Duration(1))
            return CommandServiceResponse()




def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = CommandServer()
    command_service = rospy.Service('/web_teleop/command_service', CommandService, server.parse_command)
    rospy.spin()


if __name__ == '__main__':
    main()
