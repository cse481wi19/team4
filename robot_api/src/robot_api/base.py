#! /usr/bin/env python

import geometry_msgs.msg
import rospy
import math
import copy
import nav_msgs.msg
import tf.transformations as tft
import numpy as np

ANGLELIMIT = 0.01

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=5)
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        self._odom = None

    def _odom_callback(self, msg):
        # TODO: do something
        self._odom = msg.pose.pose

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        # TODO: Fill out msg
        # TODO: Publish msg
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._pub.publish(twist)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        self.move(0,0)

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        while self._odom is None:
            rospy.sleep(0.1)

        start = copy.deepcopy(self._odom)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        distanceCur = math.sqrt((start.position.x - self._odom.position.x)**2 + (start.position.y - self._odom.position.y)**2)
        while distanceCur < distance:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
            distanceCur = math.sqrt((start.position.x - self._odom.position.x)**2 + (start.position.y - self._odom.position.y)**2)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        while self._odom is None:
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._odom)
        direction = -1 if angular_distance < 0 else 1
        mat = tft.quaternion_matrix([self._odom.orientation.x, self._odom.orientation.y, self._odom.orientation.z, self._odom.orientation.w])
        angleCur = math.atan2(mat[1, 0], mat[0, 0])
        angleEnd = (angleCur + angular_distance)
        print(str(angleCur) + "    " + str(angleEnd))
        if math.fabs(angleEnd) > math.pi:
            angleEnd -= direction*(2*math.pi)
        # if angleEnd > math.pi:
        #     angleEnd -= (2*math.pi)
        # if angleEnd < -math.pi:
        #     angleEnd += (2*math.pi)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while not self.reachGoal(angleCur, angleEnd, direction):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(0, direction * speed)
            rate.sleep()    
            mat = tft.quaternion_matrix([self._odom.orientation.x, self._odom.orientation.y, self._odom.orientation.z, self._odom.orientation.w])
            angleCur = math.atan2(mat[1, 0], mat[0, 0])

    def reachGoal(self, angleCur, angleEnd, direction):
        print(str(angleCur) + "    " + str(angleEnd))
        if angleCur*angleEnd < 0:
            return False
        if direction > 0:
            angleCur += ANGLELIMIT;
            if angleEnd > 0:
                if angleCur > angleEnd:
                    return True
            else:
                if angleCur > angleEnd:
                    return True
        else:
            angleCur -= ANGLELIMIT;
            if angleEnd > 0:
                if angleCur < angleEnd:
                    return True
            else:
                if angleCur < angleEnd:
                    return True

        return False





