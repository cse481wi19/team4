#! /usr/bin/env python

from people_msgs.msg import PositionMeasurementArray
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
    	self.sub = rospy.Subscriber('face_detector/people_tracker_measurements', PositionMeasurementArray, callback=self.callback)
    	self.arr = PositionMeasurementArray()

    def callback(self, msg):
        self.arr = msg
        rospy.logerr(len(self.arr.people))


def main():
    rospy.init_node('hallucinated_reach')
    wait_for_time()
                                                                               
    reader = ArTagReader()
    rospy.spin()


if __name__ == '__main__':
    main()

