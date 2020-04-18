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
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
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
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

class Status(Enum):
    NONE = 0
    SUCCESS = 1
    SPACE_EXCEEDED = 2
    TIME_EXCEEDED = 3
    ABORTION = 4
    COLLISION = 5

class Experiments():
    def __init__(self):

        rospy.init_node('Experiments')

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

        # variables
        self.rate = rospy.Rate(10)
        self.freecells_start = []
        self.freecells_goal = []
        #
        self.bag = None
        self.clock = None
        self.tf = []
        self.robot_pose = None
        # self.bsf = None
        # self.bsb = None
        # self.pc = None
        #
        self.experiment_finished = False
        self.robot_updated = False

        # publishers
        self.pub_initpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_freecells_start = rospy.Publisher('/freecells_start', MarkerArray, queue_size=10)
        self.pub_freecells_goal = rospy.Publisher('/freecells_goal', MarkerArray, queue_size=10)

        # subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        # rospy.Subscriber('/map', OccupancyGrid, self.map1_callback)
        # rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map2_callback)
        # rospy.Subscriber('/collision', Bool, self.collision_callback)
        # rospy.Subscriber('/base_scan_front', LaserScan, self.bsf_callback)
        # rospy.Subscriber('/base_scan_back', LaserScan, self.bsb_callback)
        # rospy.Subscriber('/xtion/depth/points', PointCloud2, self.pc_callback)

        # services
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

        # log
        rospy.loginfo('world_model_name: ' + self.world_model_name)
        rospy.loginfo('robot: ' + self.robot_model_name)
        rospy.loginfo('robot vel: ' + str(self.robot_vel))
        rospy.loginfo('space factor tolerance: ' + str(self.space_factor_tolerance))
        rospy.loginfo('time factor tolerance: ' + str(self.time_factor_tolerance))
        rospy.loginfo('max experiments: ' + str(self.max_experiments))
        print ('')

    #### calbacks ####

    def model_callback(self, data):
        try:
            index = data.name.index(self.robot_model_name)
            self.robot_pose = data.pose[index]
            self.robot_updated = True
        except ValueError as e:
            pass

    def movebase_callback(self, state, result):
        self.experiment_finished = True
        # if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
        if (state == 3):
            self.status = Status.SUCCESS
        else:
            self.status = Status.ABORTION

    def tf_callback(self, data):
        self.tf = data
    def clock_callback(self, data):
        self.clock = data
    # def bsf_callback(self, data):
    #     self.bsf = data
    # def bsb_callback(self, data):
    #     self.bsb = data
    # def pc_callback(self, data):
    #     self.pc = data

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
    # def collision_callback(self, data):
    #     if (data.data == True):
    #         self.experiment_finished = True
    #         self.status = Status.COLLISION
    #


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

        # experiments loop
        for i in range(0, self.max_experiments):
            rospy.loginfo('preparing experiment %i/%i' % (i+1, self.max_experiments))

            # init bag
            self.bag = rosbag.Bag(self.path_bags_storage + "/" + str(i)+'.bag', 'w')

            # reset variables
            self.experiment_finished = False
            self.status = Status.NONE

            # set start and goal random
            (start, goal) = self.get_start_and_goal_random()
            rospy.loginfo('Start'
            + '(x=' + str(start.pose.position.x)
            + ',y=' + str(start.pose.position.x)
            + ',ang=' + str(start.pose.orientation.z) + ')')
            rospy.loginfo('Goal'
            + '(x=' + str(goal.pose.position.x)
            + ',y=' + str(goal.pose.position.y)
            + ',ang=' + str(goal.pose.orientation.z) + ')')

            # paths
            path_plan = Path(Header(0,rospy.Time.now(),"/map"),[])
            path_executed = Path(Header(0,rospy.Time.now(),"/map"),[])

            # Finding a path plan
            rospy.loginfo('Finding a path plan...')
            while(len(path_plan.poses) is 0):
                path_plan.poses = self.make_plan(start, goal, 0.1).plan.poses
            rospy.loginfo('Path plan size: ' + str(len(path_plan.poses)))

            # Reset world
            world_model = ModelState()
            world_model.model_name = self.world_model_name
            self.model_reposition(world_model)
            self.rate.sleep()

            # Reset robot
            robot_model = ModelState()
            robot_model.model_name = self.robot_model_name
            robot_model.pose = start.pose
            self.model_reposition(robot_model)
            self.rate.sleep()

            # # Reset robot position
            # initpose = PoseWithCovarianceStamped()
            # initpose.header = Header(0,rospy.Time.now(),'map')
            # initpose.pose.pose = start.pose
            # self.pub_initpose.publish(initpose)
            # self.rate.sleep()

            # clear costmaps
            self.clear_costmaps()
            self.rate.sleep()

            # set min dist and time to reach destination
            (space_min, time_min) = self.get_min_dist_time(path_plan)
            rospy.loginfo('Space min: ' + str(space_min) + ' meters')
            rospy.loginfo('Time min: ' + str(time_min) + ' seconds')

            # set max dist and time to reach destination
            space_max = space_min*self.space_factor_tolerance
            time_max = time_min*self.time_factor_tolerance
            rospy.loginfo('Space max: ' + str(space_max) + ' meters')
            rospy.loginfo('Time max: ' + str(time_max) + ' seconds')

            # bag
            self.bag.write('/clock', self.clock)
            self.bag.write('/start', start)
            self.bag.write('/goal', goal)
            self.bag.write('/path_plan', path_plan)

            # Waiting for robot update
            rospy.loginfo('Waiting for robot update...')
            self.robot_updated = False;
            while not rospy.is_shutdown():
                self.rate.sleep()
                if(self.robot_updated):
                    break

            # start experiment
            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))

            # send commando to move_base
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header = goal.header
            mb_goal.target_pose.pose = goal.pose
            self.move_base.send_goal(mb_goal, done_cb=self.movebase_callback)
            # self.move_base.wait_for_result()

            delta_space = []
            total_space = 0
            s_begin = start.pose.position
            delta_space.append(s_begin)


            delta_time = []
            total_time = 0
            t_begin = rospy.Time.now()
            delta_time.append(t_begin)

            # loop
            step = 0
            while not rospy.is_shutdown():

                ps = PoseStamped(Header(step,rospy.Time.now(),"/map"),self.robot_pose)
                path_executed.poses.append(ps)

                # update space
                s_now = self.robot_pose.position
                delta_space.append(math.sqrt(
                    pow((s_now.x - s_begin.x), 2)+
                    pow((s_now.y - s_begin.y), 2)))
                total_space += delta_space[-1]
                s_begin = s_now;

                # update time
                t_now = rospy.Time.now()
                delta_time.append(t_now - t_begin)
                total_time += delta_time[-1].to_sec()
                t_begin = t_now;

                # check space restriction
                if(total_space > space_max):
                    self.experiment_finished = True
                    self.status = Status.SPACE_EXCEEDED

                # check time restriction
                if(total_time > time_max):
                    self.experiment_finished = True
                    self.status = Status.TIME_EXCEEDED

                # bag
                self.bag.write('/clock', self.clock)
                self.bag.write('/tf', self.tf)
                # self.bag.write('/path_executed', path_executed)
                # self.bag.write('/base_scan_front', self.bsf)
                # self.bag.write('/base_scan_back', self.bsb)
                # self.bag.write('/xtion/depth/points', self.pc)

                # break
                if(self.experiment_finished):
                     break

                # step increment
                step = step+1


            # close bag
            self.bag.write('/path_executed', path_executed)
            self.bag.close()

            # log
            rospy.loginfo('Space elapsed: ' + str(total_space) + ' meters')
            rospy.loginfo('Time elapsed: ' + str(total_time) + ' seconds')
            rospy.loginfo('Status: ' + self.status.name)
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments))
            print ('')

            # stop tracking end cancel finished move_base goal
            self.move_base.cancel_all_goals()
            self.move_base.stop_tracking_goal();
            self.rate.sleep()


if __name__ == '__main__':
    ex = Experiments()
    ex.start()
