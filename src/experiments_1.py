#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

class Experiments():
    """docstring for Experiments"""
    def __init__(self):

        rospy.init_node('Experiments')
        self.rate = rospy.Rate(10)

        # parameters
        self.environment = rospy.get_param('phd_experiments/environment', '')
        self.robot_name = rospy.get_param('phd_experiments/robot_name', '')
        self.max_experiments = rospy.get_param('phd_experiments/max_experiments', 10)

        # services
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_reposition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # actions
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

    def movebase_callback(self, state, result):
        pass
        # self.experiment_finished = True
        # # if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
        # if (state == 3):
        #     self.status = Status.SUCCESS
        # else:
        #     self.status = Status.ABORTION

    def start(self):
        rospy.loginfo('max experiments: ' + str(self.max_experiments))

        # experiments loop
        for i in range(0, self.max_experiments):

            # Reset environment
            ms_environment = ModelState()
            ms_environment.model_name = self.environment
            ms_environment.reference_frame = 'map'
            self.model_reposition(ms_environment)
            self.rate.sleep()

            # Reset robot
            ms_robot = ModelState()
            ms_robot.model_name = self.robot_name
            ms_robot.reference_frame = 'map'
            ms_robot.pose.position.x = 0.0
            ms_robot.pose.position.y = 0.0
            ms_robot.pose.position.z = 0.1
            ms_robot.pose.orientation.x = 0.0
            ms_robot.pose.orientation.y = 0.0
            ms_robot.pose.orientation.z = 0.0
            ms_robot.pose.orientation.w = 1.0
            self.model_reposition(ms_robot)
            self.rate.sleep()

            # clear costmaps
            self.clear_costmaps()
            self.rate.sleep()

            # start
            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))

            # send commando to move_base
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 12.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.1
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            self.move_base.send_goal(goal, done_cb=self.movebase_callback)
            self.move_base.wait_for_result()

            # finish
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments))

            # stop tracking end cancel finished move_base goal
            self.move_base.cancel_all_goals()
            self.move_base.stop_tracking_goal();


if __name__ == '__main__':
    ex = Experiments()
    ex.start()
