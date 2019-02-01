#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry


import math
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def distance(p1, p2):
    """Returns the distance between two Points/Vector3s.
    """
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class NavPath(object):

    def __init__(self, publisher):
        self._path = []
        self._publisher = publisher

    def callback(self, msg):
        point = msg.pose.pose.position
        if (len(self._path) == 0 or
                distance(self._path[-1], point) > 0.3):
            self._path.append(point)
            
            self.viz_path()

    def viz_path(self):
        msg = Marker(
            type=Marker.LINE_STRIP,
            ns='path',
            id=len(self._path),
            points=self._path,
            scale=Vector3(0.06, 0.06, 0.06),
            header=Header(frame_id='odom'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self._publisher.publish(msg)


def main():
    rospy.init_node('path_viz')
    wait_for_time()
    marker_publisher = rospy.Publisher(
        'visualization_marker', Marker, queue_size=5)
    path = NavPath(marker_publisher)
    rospy.Subscriber('odom', Odometry, path.callback)

    rospy.spin()


if __name__ == '__main__':
    main()