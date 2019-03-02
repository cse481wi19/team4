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

class Server(object):
    def __init__(self, database, arm, gripper):
        self.db = database
        self.db.load()
        self.arm = arm
        self.gripper = gripper
        self.reader = ArTagReader()
        self.sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self.reader.callback)
        self.listener = tf.TransformListener()

        self.savedBefore = False

    # Currently only add pose relative to tag instead of base_link
    def add(self, name, tag):
        # Calculate the coordinate of wrist based on base_link frame
        if not self.savedBefore:
            self.reader.update()
            self.savedBefore = True
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
        elif tag == 'open':
        	self.db.add(name, None, tag)
        elif tag == 'close':
        	self.db.add(name, None, tag)
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
        if self.db.get(name) is not None:
            for offset, tag in self.db.get(name):
                if tag == 'base_link':
                    self.arm.move_to_pose(offset)
                elif tag == 'open':
        	        self.gripper.open()
                elif tag == 'close':
        	        self.gripper.close()
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

class Database(object):
    def __init__(self):
        self._markers = {}
        self.db_path = 'marker_db.p'

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


def main():
    rospy.init_node('pdb')
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
        print("\thelp: List all commands")

    gripper = robot_api.Gripper()
    arm = robot_api.Arm()
    database = Database()
    facedetector = robot_api.FaceDetector()
    server = Server(database, arm, gripper)
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
            ps.pose.position.x = 0.5
            ps.pose.position.y = 0.5
            ps.pose.position.z = 0.75
            ps.header.frame_id = 'base_link'
            arm.move_to_pose(ps)

        elif command[:4] == "move":
            if len(command[5:]) == 0:
                print("No coordinate given")
            l = command[5:].split()
            if len(l) != 3 and (len(l) == 1 and l[0] != "face"):
                print("Argument format: move x y z or move face") 
            elif len(l) == 3:
                ps = PoseStamped()
                ps.pose.position.x = float(l[0])
                ps.pose.position.y = float(l[1])
                ps.pose.position.z = float(l[2])
                ps.header.frame_id = 'base_link'
                arm.move_to_pose(ps)
            else:
                if facedetector.pose is not None:
                    arm.move_to_pose(facedetector.pose)
                else:
                    print("No face is detected.")

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

        else:
            print("Invalid command, please re-enter :)")

    def handle_shutdown():
        database.save()    

    rospy.on_shutdown(handle_shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()
      
      
