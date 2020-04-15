#!/usr/bin/env python

import tf
import cv2
import math
import time
import rospy
import numpy
import rosbag
import random
import actionlib
from PIL import Image
from enum import Enum

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class Status(Enum):
    NONE = 0
    SUCCESS = 1
    SPACE_EXCEEDED = 2
    TIME_EXCEEDED = 3
    ABORTION = 4
    COLLISION = 5

class Data():
    """docstring for Data."""
    def __init__(self):

        # pre experiment info
        self.start = None
        self.goal = None
        self.plan = None
        self.space_min = None
        self.time_min = None

        # in experiment info
        self.odom = []
        self.v_space_elapsed = []
        self.v_time_elapsed = []

        # pos experiment info
        self.status = None
        self.space_elapsed = None
        self.time_elapsed = None

class Experiments():
    """docstring for Experiments"""
    def __init__(self):
        rospy.init_node('Experiments')
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.robot_position = None
        self.freecells_start = []
        self.freecells_goal = []
        self.bag = None

        # parameters
        self.world_model_name = rospy.get_param('social_experiments/world_model_name', '')
        self.robot_model_name = rospy.get_param('social_experiments/robot_model_name', '')
        self.max_experiments = rospy.get_param('social_experiments/max_experiments', 100)
        self.path_bags_storage = rospy.get_param('social_experiments/path_bags_storage', '')
        self.robot_vel = rospy.get_param('social_experiments/robot_vel', 0.3)
        self.space_factor_tolerance = rospy.get_param('social_experiments/space_factor_tolerance', 5)
        self.time_factor_tolerance = rospy.get_param('social_experiments/time_factor_tolerance', 5)
        self.path_img_freecells_start = rospy.get_param('social_experiments/path_img_freecells_start', '')
        self.path_img_freecells_goal = rospy.get_param('social_experiments/path_img_freecells_goal', '')

        # publishers
        self.pub_initpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_freecells_start = rospy.Publisher('/freecells_start', MarkerArray, queue_size=10)
        self.pub_freecells_goal = rospy.Publisher('/freecells_goal', MarkerArray, queue_size=10)

        # # subscribers
        # rospy.Subscriber('/map', OccupancyGrid, self.map1_callback)
        # rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map2_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        # rospy.Subscriber('/collision', Bool, self.collision_callback)
        #
        # # services
        rospy.loginfo('Wating for "/gazebo/set_model_state" service')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_reposition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo('Wating for "/move_base/clear_costmaps" service')
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo('Wating for "/move_base/NavfnROS/make_plan" service')
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        self.make_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

        # actions
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

    #### calbacks ####

    # def map1_callback(self, data):
    #     # self.map1 = data.data
    #     img = Image.new('RGB', (data.info.width, data.info.height))
    #     img.putdata(data.data)
    #     img.save('map.png')
    #
    # def map2_callback(self, data):
    #     # self.map2 = data.data
    #     img = Image.new('RGB', (data.info.width, data.info.height))
    #     img.putdata(data.data)
    #     img.save('costmap.png')
    #
    #
    #
    def model_callback(self, data):
        try:
            index = data.name.index(self.robot_model_name)
            self.actual_robot_position = data.pose[index].position
            self.robot_updated = True
        except ValueError as e:
            pass
    # def collision_callback(self, data):
    #     if (data.data == True):
    #         self.experiment_finished = True
    #         self.status = Status.COLLISION
    #
    def movebase_callback(self, state, result):
        self.experiment_finished = True
        # if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
        if (state == 3):
            self.status = Status.SUCCESS
        else:
            self.status = Status.ABORTION


    ### functions ####

    def get_start_and_goal_random(self):

        # get robot random start pose
        start = Pose()
        start_angle = random.randint(0,360)
        start_index = random.randint(0,len(self.freecells_start))
        start_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(start_angle))
        start.position.x = self.freecells_start[start_index][0]
        start.position.y = self.freecells_start[start_index][1]
        start.position.z = 0.1
        start.orientation.x = start_quaternion[0]
        start.orientation.y = start_quaternion[1]
        start.orientation.z = start_quaternion[2]
        start.orientation.w = start_quaternion[3]
        start_stamped = PoseStamped(Header(0,rospy.Time.now(),'map'), start)

        # get robot random goal pose
        goal = Pose()
        goal_angle = random.randint(0,360)
        goal_index = random.randint(0,len(self.freecells_goal))
        goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(goal_angle))
        goal.position.x = self.freecells_goal[goal_index][0]
        goal.position.y = self.freecells_goal[goal_index][1]
        goal.position.z = 0.1
        goal.orientation.x = goal_quaternion[0]
        goal.orientation.y = goal_quaternion[1]
        goal.orientation.z = goal_quaternion[2]
        goal.orientation.w = goal_quaternion[3]
        goal_stamped = PoseStamped(Header(0,rospy.Time.now(),'map'), goal)

        return (start_stamped, goal_stamped)

    def get_min_dist_time(self, plan):

        space_min = 0
        time_min = 0

        p = Point()
        p.x = plan.poses[0].pose.position.x
        p.y = plan.poses[0].pose.position.y

        # set minimum space to reach a goal
        space_min += math.sqrt(pow((plan.poses[0].pose.position.x - p.x), 2)+
                          pow((plan.poses[0].pose.position.y - p.y), 2))
        for k in range(1,len(plan.poses)):
            space_min += math.sqrt(pow((plan.poses[k].pose.position.x
                                 - plan.poses[k-1].pose.position.x), 2)+
                              pow((plan.poses[k].pose.position.y
                                 - plan.poses[k-1].pose.position.y), 2))

        # set minimum time to reach a goal
        time_min = space_min/self.robot_vel;

        return (space_min, time_min)

    def get_freecells_markers(self, freecells):
        markerArray = MarkerArray()
        id = 0
        for fc in freecells:

            marker = Marker()
            marker.id = id
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = fc[0]
            marker.pose.position.y = fc[1]
            marker.pose.position.z = 0.0

            markerArray.markers.append(marker)
            id += 1

        # return the MarkerArray
        return markerArray

    def start(self):
        rospy.loginfo('world_model_name: ' + self.world_model_name)
        rospy.loginfo('robot: ' + self.robot_model_name)
        rospy.loginfo('robot vel: ' + str(self.robot_vel))
        rospy.loginfo('space factor tolerance: ' + str(self.space_factor_tolerance))
        rospy.loginfo('time factor tolerance: ' + str(self.time_factor_tolerance))
        rospy.loginfo('max experiments: ' + str(self.max_experiments) + '\n')


        # get freecells
        img_freecells_start = cv2.imread(self.path_img_freecells_start,0)
        img_freecells_goal = cv2.imread(self.path_img_freecells_goal,0)
        (h_s, w_s) = img_freecells_start.shape[:2]
        (h_g, w_g) = img_freecells_goal.shape[:2]
        MS = cv2.getRotationMatrix2D((w_s/2, h_s/2), -90, 1.0)
        MG = cv2.getRotationMatrix2D((w_g/2, h_g/2), -90, 1.0)
        img_freecells_start = cv2.warpAffine(img_freecells_start, MS, (h_s,w_s))
        img_freecells_goal = cv2.warpAffine(img_freecells_goal, MG, (h_g,w_g))
        a_s = numpy.argwhere(img_freecells_start == 255)
        a_g = numpy.argwhere(img_freecells_goal == 255)
        self.freecells_start = a_s*0.05 - 100
        self.freecells_goal = a_g*0.05 - 100
        start_marker_array = self.get_freecells_markers(self.freecells_start)
        goal_marker_array = self.get_freecells_markers(self.freecells_goal)
        self.pub_freecells_start.publish(start_marker_array)
        self.pub_freecells_goal.publish(goal_marker_array)

        # experiments loop
        for i in range(0, self.max_experiments):

            # init bag
            self.bag = rosbag.Bag(self.path_bags_storage + "/" + str(i)+'.bag', 'w')

            # prepare experiment
            rospy.loginfo('prepare experiment %i/%i' % (i+1, self.max_experiments))

            # store experiments
            data = Data()

            # reset variables
            self.experiment_finished = False
            self.status = Status.NONE

            # set start and goal random
            (start, goal) = self.get_start_and_goal_random()
            data.start = start
            data.goal = goal
            rospy.loginfo('Start'
            + '(x=' + str(data.start.pose.position.x)
            + ',y=' + str(data.start.pose.position.x)
            + ',ang=' + str(data.start.pose.orientation.z) + ')')
            rospy.loginfo('Goal'
            + '(x=' + str(data.goal.pose.position.x)
            + ',y=' + str(data.goal.pose.position.y)
            + ',ang=' + str(data.goal.pose.orientation.z) + ')')
            self.bag.write('start', data.start)
            self.bag.write('goal', data.goal)

            # get a plan
            rospy.loginfo('Finding a plan...')
            plan = Path()
            while(len(plan.poses) is 0):
                plan = self.make_plan(data.start, data.goal, 0.0).plan
            data.plan= plan
            rospy.loginfo('plan size: ' + str(len(plan.poses)))
            self.bag.write('plan', data.plan)

            # Reset world
            world_model = ModelState()
            world_model.model_name = self.world_model_name
            self.model_reposition(world_model)
            self.rate.sleep()

            # Reset robot
            robot_model = ModelState()
            robot_model.model_name = self.robot_model_name
            robot_model.pose = data.start.pose
            self.model_reposition(robot_model)
            self.rate.sleep()

            # Reset robot amcl position
            initpose = PoseWithCovarianceStamped()
            initpose.header = Header(0,rospy.Time.now(),'/map')
            initpose.pose.pose = data.start.pose
            self.pub_initpose.publish(initpose)
            self.rate.sleep()

            # clear costmaps
            self.clear_costmaps()
            self.rate.sleep()

            # set min dist and time to reach destination
            (space_min, time_min) = self.get_min_dist_time(plan)
            data.space_min = space_min
            data.time_min = time_min
            rospy.loginfo('Space min: ' + str(data.space_min) + ' meters')
            rospy.loginfo('Time min: ' + str(data.time_min) + ' seconds')

            rospy.loginfo('Waiting for robot position update...')
            self.robot_updated = False;
            while not rospy.is_shutdown():
                self.rate.sleep()
                if(self.robot_updated):
                    break


            # start
            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))

            # send commando to move_base
            goal = MoveBaseGoal()
            goal.target_pose.header = data.goal.header
            goal.target_pose.pose = data.goal.pose
            self.move_base.send_goal(goal, done_cb=self.movebase_callback)
            # self.move_base.wait_for_result()

            #
            space_elapsed = 0
            s_begin = Point()
            s_begin.x = data.start.pose.position.x
            s_begin.y = data.start.pose.position.y
            s_begin.z = 0.0
            data.v_space_elapsed.append(s_begin)

            #
            time_elapsed = 0
            t_begin = time.time()
            data.v_time_elapsed.append(t_begin)

            # loop
            k = 0
            while not rospy.is_shutdown():
                # self.rate.sleep()

		        # update localization error
            	(trans,rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                p = PoseStamped(
                    Header(k,rospy.Time(0),'/map'), PoseStamped(
                    Point(trans[0],trans[1],trans[2]),
                    Quaternion(rot[0],rot[1],rot[2],rot[3])))
                data.odom.append(p)

                # update space
                s_now = self.actual_robot_position
                space_elapsed += math.sqrt(pow((s_now.x - s_begin.x), 2)+
                                           pow((s_now.y - s_begin.y), 2))
                s_begin = s_now;
                data.v_space_elapsed.append(s_begin)
                self.bag.write('position', self.actual_robot_position)


                # update time
                t_now = time.time()
                t_elapsed = t_now - t_begin
                time_elapsed = t_elapsed
                # t_begin = t_now;
                data.v_time_elapsed.append(time_elapsed)

                # check space restriction
                space_max = data.space_min*self.space_factor_tolerance
                if(space_elapsed > space_max):
                    self.experiment_finished = True
                    self.status = Status.SPACE_EXCEEDED

                # check time restriction
                time_max = data.time_min*self.time_factor_tolerance
                if(time_elapsed > time_max):
                    self.experiment_finished = True
                    self.status = Status.TIME_EXCEEDED

                # break
                if(self.experiment_finished):
                     break

                k = k+1

            # set pos info
            data.space_elapsed = space_elapsed
            data.time_elapsed = time_elapsed
            data.status = self.status

            # print pos info
            rospy.loginfo('Space elapsed: ' + str(data.space_elapsed) + ' meters')
            rospy.loginfo('Time elapsed: ' + str(data.time_elapsed) + ' seconds')
            rospy.loginfo('Status: ' + data.status.name)

            # finish
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments) + '\n')
            self.bag.close()

            # stop tracking end cancel finished move_base goal
            self.move_base.cancel_all_goals()
            self.move_base.stop_tracking_goal();


if __name__ == '__main__':
    ex = Experiments()
    ex.start()
