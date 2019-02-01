#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy
import robot_api
import math


def wait_for_time():
	"""Wait for simulated time to begin.
	"""
	while rospy.Time().now().to_sec() == 0:
		pass



class Interactive(object):
	def __init__(self, server, x, y, name):
		self._server = server
		self._x = x
		self._y = y
		self._name = name

	def make(self):
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		int_marker.name = self._name
		int_marker.description = self._name
		int_marker.pose.position.x = self._x
		int_marker.pose.position.y = self._y
		int_marker.pose.orientation.w = 1

		box_marker = Marker()
		box_marker.type = Marker.CUBE
		box_marker.pose.orientation.w = 1
		box_marker.scale.x = 0.45
		box_marker.scale.y = 0.45
		box_marker.scale.z = 0.45
		box_marker.color.r = 0.0
		box_marker.color.g = 0.5
		box_marker.color.b = 0.5
		box_marker.color.a = 1.0

		button_control = InteractiveMarkerControl()
		button_control.interaction_mode = InteractiveMarkerControl.BUTTON
		button_control.always_visible = True
		button_control.markers.append(box_marker)
		int_marker.controls.append(button_control)

		self._server.insert(int_marker, self.handle_viz_input)
		self._server.applyChanges()


	def handle_viz_input(self, input):
		base = robot_api.Base()
		if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
			rospy.loginfo(input.marker_name + ' was clicked.')
			if (input.marker_name == 'forward'): 
				base.go_forward(0.5)
			elif (input.marker_name == 'rotate clockwise'):
				base.turn(-30.0/180*math.pi)
			elif (input.marker_name == 'rotate counter-clockwise'):
				base.turn(30.0/180*math.pi)
				




def main():
	rospy.init_node('interactive_marker_demo')
	wait_for_time()

	# marker_publisher = rospy.Publisher('', Marker, queue_size=5)

	server = InteractiveMarkerServer("simple_marker")
	marker1 = Interactive(server, 1, 0, 'forward')
	marker2 = Interactive(server, 0, 1, 'rotate counter-clockwise')
	marker3 = Interactive(server, 0, -1, 'rotate clockwise')
	marker1.make()
	marker2.make()
	marker3.make()
	rospy.spin()


if __name__ == '__main__':
    main()