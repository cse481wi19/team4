#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('gripper_tf_listener')
    while rospy.Time().now().to_sec() == 0:
        pass
    listener = tf.TransformListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
            rospy.loginfo('{} {}'.format(trans, rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()