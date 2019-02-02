#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy
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


    def update_pose(self, input):
        if (input.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            self._db.add(feedback.marker_name, feedback.pose)
            self._server.setPose(feedback.marker_name, feedback.pose)
            self._server.applyChanges()


class Server(object):
    def __init__(self):
        self.server = InteractiveMarkerServer("map_annotator/map_poses")
        self.publisher = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=1)
        self.poses = PoseNames()

        self.db = Database()


    def load(self):
        self._db.load()
        for name in self.db.list():
            self.create(name)

    def handleAction(self, msg):
        if msg.command == "save":
            print(msg.name)
            self.create(msg.name)
        # elif msg.command == "delete":

        # elif msg.command == "goto":
        #     goto()



    def create(self, name):
        pose = self.db.get(name)
        if pose is None:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
        marker = Annotator(self.server, pose, name, self._db) # add database for create and callback
        marker.make() # add into database
        self.poses.names.append(name)
        self.publisher.publish(self.poses)


class Database(object):
    DB_PATH = './marker_db.data'

    def __init__(self):
        self._markers = {}

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
            with open(DB_PATH, 'r') as f:
                self._markers = pickle.load(f)
        except IOError as e:
            rospy.logwarn('No storage information: {}'.format(e))


    def save(self):
        try:
            with open(DB_PATH, 'w') as f:
                pickle.dump(self._markers, f)
        except IOError as e:
            rospy.logwarn('No storage information: {}'.format(e))


def main():
    rospy.init_node('interactive_marker_demo')
    wait_for_time()

	# marker_publisher = rospy.Publisher('', Marker, queue_size=5)

    server = Server()
    server.load()

    saveSub = rospy.Subscriber("map_annotator/user_actions", UserAction, server.handleAction)
    rospy.spin()


if __name__ == '__main__':
    main()
			
			