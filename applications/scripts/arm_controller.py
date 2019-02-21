#! /usr/bin/env python
from robot_controllers_msgs.msg import QueryControllerStatesAction, 
                                        QueryControllerStatesGoal,
                                       ControllerState
import rospy
import robot_api

def print_usage():
    print 'Usage:'
    print '    rosrun applications arm_controller.py start'
    print '    rosrun applications arm_controller.py stop'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_controller')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) != 2:
        print_usage()
        return
    command = argv[1]
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(ACTION_SERVER, robot_controllers_msgs.QueryControllerStates)
    self._controller_client.send_goal(goal)
    self._controller_client.wait_for_result()
    if command == 'start':
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        lient.send_goal(goal)
        client.wait_for_result()
    elif command == 'stop':
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        lient.send_goal(goal)
        client.wait_for_result()
    

if __name__ == '__main__':
    main()