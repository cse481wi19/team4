
#! /usr/bin/env python

import robot_api
import rospy
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import OrientationConstraint



def wait_for_time():
	"""Wait for simulated time to begin.
	"""
	while rospy.Time().now().to_sec() == 0:
		pass


def main():
	rospy.init_node('arm_obstacle_demo')
	wait_for_time()
	argv = rospy.myargv()

	planning_scene = PlanningSceneInterface('base_link')
	planning_scene.clear()
	planning_scene.removeAttachedObject('tray')

    # Create table obstacle
	planning_scene.removeCollisionObject('table')
	table_size_x = 0.5
	table_size_y = 1
	table_size_z = 0.03
	table_x = 0.8
	table_y = 0
	table_z = 0.6
	planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
	                      table_x, table_y, table_z)

	# Create divider obstacle
	planning_scene.removeCollisionObject('divider')
	size_x = 0.3 
	size_y = 0.01
	size_z = 0.4 
	x = table_x - (table_size_x / 2) + (size_x / 2)
	y = 0 
	z = table_z + (table_size_z / 2) + (size_z / 2)
	planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

	pose1 = PoseStamped()
	pose1.header.frame_id = 'base_link'
	pose1.pose.position.x = 0.5
	pose1.pose.position.y = -0.3
	pose1.pose.position.z = 0.75
	pose1.pose.orientation.w = 1

	pose2 = PoseStamped()
	pose2.header.frame_id = 'base_link'
	pose2.pose.position.x = 0.5
	pose2.pose.position.y = 0.3
	pose2.pose.position.z = 0.75
	pose2.pose.orientation.w = 1

	oc = OrientationConstraint()
	oc.header.frame_id = 'base_link'
	oc.link_name = 'wrist_roll_link'
	oc.orientation.w = 1
	oc.absolute_x_axis_tolerance = 0.1
	oc.absolute_y_axis_tolerance = 0.1
	oc.absolute_z_axis_tolerance = 3.14
	oc.weight = 1.0

	arm = robot_api.Arm()
	def shutdown():
	    arm.cancel_all_goals()
	rospy.on_shutdown(shutdown)

	kwargs = {
	    'allowed_planning_time': 100,
	    'execution_timeout': 100,
	    'num_planning_attempts': 1,
	    'replan_attempts': 5,
	    'replan': True,
	    'orientation_constraint': None
	}

	error = arm.move_to_pose(pose1, **kwargs)
	if error is not None:
	    rospy.logerr('Pose 1 failed: {}'.format(error))
	else:
	    rospy.loginfo('Pose 1 succeeded')
	    frame_attached_to = 'gripper_link'
	    frames_okay_to_collide_with = [
	        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
	    ]
	    planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
	                             frame_attached_to, frames_okay_to_collide_with)
	    planning_scene.setColor('tray', 1, 0, 1)
	    planning_scene.sendColors()
	rospy.sleep(1)
	kwargs['orientation_constraint'] = oc
	error = arm.move_to_pose(pose2, **kwargs)
	if error is not None:
	    rospy.logerr('Pose 2 failed: {}'.format(error))
	else:
	    rospy.loginfo('Pose 2 succeeded')

	planning_scene.removeCollisionObject('table')
	planning_scene.removeCollisionObject('divider')
	# planning_scene.removeAttachedObject('tray')



if __name__ == '__main__':
	main()