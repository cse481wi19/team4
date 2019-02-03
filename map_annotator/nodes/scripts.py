#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy
import actionlib
import robot_api
import math
import pickle
from map_annotator.msg import PoseNames, UserAction

def wait_for_time():
	"""Wait for simulated time to begin.
	"""
	while rospy.Time().now().to_sec() == 0:
		pass

class Annotator(object):
    def __init__(self, server, pose, name, database):
        self._server = server
        self._x = pose.position.x
        self._y = pose.position.y
        self._name = name

        self._db = database

    def make(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = self._name
        int_marker.description = self._name
        int_marker.pose.position.x = self._x
        int_marker.pose.position.y = self._y
        int_marker.pose.orientation.w = 1

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.45
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.orientation.w = 1
        text_marker.pose.position.z = 1.5
        text_marker.scale.x = 0.2
        text_marker.scale.y = 0.2
        text_marker.scale.z = 0.2
        text_marker.color.r = 0.5
        text_marker.color.g = 0.5
        text_marker.color.b = 0.5
        text_marker.color.a = 1
        text_marker.text = self._name

        arrow_control = InteractiveMarkerControl()
        arrow_control.orientation.w = 1
        arrow_control.orientation.y = 1
        arrow_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        arrow_control.markers.append(arrow_marker)
        arrow_control.markers.append(text_marker)
        arrow_control.always_visible = True
        int_marker.controls.append(arrow_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.y = 1
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        int_marker.controls.append(control)

        self._db.add(int_marker.name, int_marker.pose)

        self._server.insert(int_marker, self.update_pose)
        self._server.applyChanges()


    def update_pose(self, feedback):
        if (feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            self._db.add(feedback.marker_name, feedback.pose)
            self._server.setPose(feedback.marker_name, feedback.pose)
            self._server.applyChanges()


class Server(object):
    def __init__(self, database):
        self.server = InteractiveMarkerServer("map_annotator/map_poses")
        self.publisher = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=1)
        self.moveClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.poses = PoseNames()
        self.db = database
        self.load()
        self.poses.names.extend(self.db.list())
        self.publisher.publish(self.poses)

    def load(self):
        self.db.load()
        for name in self.db.list():
            self.create(name)

    def handleAction(self, msg):
        if msg.command == "save":
            self.create(msg.name)
        elif msg.command == "delete":
            self.delete(msg.name)
        elif msg.command == "goto":
            self.goto(msg.name)



    def create(self, name):
        pose = self.db.get(name)
        if pose is None:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
        marker = Annotator(self.server, pose, name, self.db) # add database for create and callback
        marker.make() # add into database
        self.poses.names *= 0
        self.poses.names.extend(self.db.list())
        self.publisher.publish(self.poses)

    def goto(self, name):
        pose = self.db.get(name)
        if pose is None:
            rospy.logwarn('No poses found: {}'.format(e))
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time().now()
            goal.target_pose.pose = pose
            self.moveClient.send_goal(goal)

    def delete(self, name):
        self.db.delete(name)
        self.server.erase(name)
        self.server.applyChanges()
        self.poses.names *= 0
        self.poses.names.extend(self.db.list())
        self.publisher.publish(self.poses)


class Database(object):
    

    def __init__(self):
        self._markers = {}
        self.db_path = 'marker_db.p'

    def get(self, name):
        if name in self._markers:
            return self._markers[name]
        else:
            return None


    def add(self, name, pose):
        self._markers[name] = pose


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
    rospy.init_node('interactive_marker_demo')
    wait_for_time()

	# marker_publisher = rospy.Publisher('', Marker, queue_size=5)
    database = Database()
    server = Server(database)

    saveSub = rospy.Subscriber("map_annotator/user_actions", UserAction, server.handleAction)


    print("Welcome to the map annotator!\n")
    print_help();
    
    while(true):
        command = input()
        if command is "list":
            poses = database.list()
            if poses is None:
                print("No poses\n")
            else: 
                print("Poses:\n")
                for pose in poses:
                    print("\t" + pose + "\n")
                print("\n")

        elif command[:4] is "save":
            if command[5:] is None:
                print("No name given\n")
            else: 
                serve.create(command[5:])

        elif command[:6] is "delete"
            poses = database.list()
            if command[7:] is None:
                print("No pose given\n")
            elif command[7:] not in poses:
                print("No such pose '" + command[7:] + "'\n")
            else:
                serve.delete(command[7:])

        elif command[:4] is "goto":
            poses = database.list()
            if command[5:] is None:
                print("No pose given\n")
            elif command[5:] not in poses:
                print("No such pose '" + command[5:] + "'\n")
            else:
                serve.goto(command[5:])

        elif command[:4] is "help":
            print_help();

        else:
            print("Invalid command, please re-enter :)")


    def handle_shutdown():
        poses = PoseNames()
        server.publisher.publish(poses)
        database.save()

    def print_help():
        print("Commands:\n")
        print("\tlist: List saved poses.\n")
        print("\tsave <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.\n")
        print("\tdelete <name>: Delete the pose given by <name>.\n")
        print("\tgoto <name>: Sends the robot to the pose given by <name>.\n")
        print("\thelp: Show this list of commands\n")


    rospy.on_shutdown(handle_shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()
			
			