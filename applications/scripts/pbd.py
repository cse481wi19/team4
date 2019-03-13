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

def wait_for_time():
  """Wait for simulated time to begin.
  """
  while rospy.Time().now().to_sec() == 0:
    pass




def main():
    rospy.init_node('pbd')
    wait_for_time()

    def print_help():
        print("Commands:")
        print("\tlist: List saved poses.")
        print("\tsave <name> <frame_id/gripper_status> : Save the robot's current pose as <name>")
        print("\tdelete <name>: Delete the pose given by <name>.")
        print("\trun <name>: Run the pose given by <name>.")
        print("\tmove <x> <y> <z> / face: Move the arm to the xyz position or face")
        print("\treset: This is to reset the ARM to 0.5 0.5 0.75, NOT THE ROBOT!")
        print("\trelax: Relax the arm")
        print("\tclose: Close the gripper")
        print("\topen: Open the gripper")
        print("\tquit: Quit the program")
        print("\tdump: Dump the cached databsae to local file")
        print("\thelp: List all commands")

    gripper = robot_api.Gripper()
    arm = robot_api.Arm()
    database = robot_api.Database()
    facedetector = robot_api.FaceDetector()
    fooddetector = robot_api.FoodDetector()
    server = robot_api.Manager(database, arm, gripper, facedetector, fooddetector)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    print_help()
    
    while(True):
        command = raw_input("> ")
        # print(command)
        if command == "list":
            poses = database.list()
            if len(poses) == 0:
                print("No poses")
            else: 
                print("Poses:")
                for pose in poses:
                    print("\t" + pose)

        elif command[:4] == "save":
            if len(command[5:]) == 0:
                print("No name given")
            l = command[5:].split()
            if len(l) != 2:
                print("Argument format: <Pose> <frame_id/gripper_status>") 
            else:
                server.add(l[0], l[1])

        elif command[:6] == "delete":
            poses = database.list()
            if command[7:] is None:
                print("No pose given\n")
            elif command[7:] not in poses:
                print("No such pose '" + command[7:] + "'")
            else:
                database.delete(command[7:])

        elif command[:3] == "run":
            poses = database.list()
            if len(command[4:]) == 0:
                print("No pose given")
            elif command[4:] not in poses:
                print("No such pose '" + command[5:] + "'")
            else:
                server.run(command[4:])

        elif command[:5] == "close":
            gripper.close()

        elif command[:4] == "open":
            gripper.open()

        elif command[:4] == "help":
            print_help()

        elif command[:5] == "reset":
            ps = PoseStamped()
            ps.pose.position.x = 0.46
            ps.pose.position.y = -0.43
            ps.pose.position.z = 1.05
            ps.pose.orientation.x = 0.38
            ps.pose.orientation.y = 0.08
            ps.pose.orientation.z = 0.23
            ps.pose.orientation.w = 0.89
            ps.header.frame_id = 'base_link'
            arm.move_to_pose(ps)

        elif command[:4] == "move":
            if len(command[5:]) == 0:
                print("No coordinate given")
            l = command[5:].split()
            if len(l) != 3 and (len(l) == 1 and l[0] != "face" and l[0] != "food"):
                print("Argument format: move x y z or move face") 
            elif len(l) == 3:
                ps = PoseStamped()
                ps.pose.position.x = float(l[0])
                ps.pose.position.y = float(l[1])
                ps.pose.position.z = float(l[2])
                ps.header.frame_id = 'base_link'
                arm.move_to_pose(ps)
            else:
                if l[0] == "face":
                    if facedetector.pose is not None:
                        arm.move_to_pose(facedetector.pose)
                    else:
                        print("No face is detected.")
                else:
                    if fooddetector.pose is not None:
                        arm.move_to_pose(fooddetector.pose)
                    else:
                        print("No food is detected.")

        elif command[:5] == "relax":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            controller_client.send_goal(goal)
            controller_client.wait_for_result(rospy.Duration(5)) 

        elif command[:5] == "start":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.RUNNING
            goal.updates.append(state)
            controller_client.send_goal(goal)
            controller_client.wait_for_result(rospy.Duration(1))

        elif command == 'quit':
            database.save()
            quit()

        elif command == 'dump':
            database.save()

        else:
            print("Invalid command, please re-enter :)")

    def handle_shutdown():
        database.save()    

    rospy.on_shutdown(handle_shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()
      
      
