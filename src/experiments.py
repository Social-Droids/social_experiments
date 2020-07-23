#!/usr/bin/env python

import tf
import cv2
import math
import time
import rospy
import numpy
import rosnode
# import rosbag
import random
import actionlib
# from PIL import Image
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
from people_msgs.msg import People
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from social_worlds.srv import Regions
from social_worlds.msg import Region


class Status(Enum):
    NONE = 0
    SUCCESS = 1
    SPACE_EXCEEDED = 2
    TIME_EXCEEDED = 3
    ABORTION = 4
    COLLISION = 5
    INVASION = 6

class Data():
    def __init__(self):

        # pre experiment info
        # self.start = None
        # self.goal = None
        self.checkpoints = []
        self.path_plan = []
        self.space_min = 0
        self.time_min = 0

        # pos experiment info
        self.status = None
        self.factor_array = []
        self.people_array = []
        self.localization_error_array = []
        self.path_executed = []
        self.delta_space = []
        self.delta_time = []
        self.total_space = 0
        self.total_time = 0


class Experiments():
    def __init__(self, global_planner, local_planner, world_model_name,
        robot_model_name
        # , path_img_freecells_start, path_img_freecells_goal
        ):

        self.global_planner = global_planner
        self.local_planner = local_planner

        self.world_model_name = world_model_name
        self.robot_model_name = robot_model_name
        # self.path_img_freecells_start = path_img_freecells_start
        # self.path_img_freecells_goal = path_img_freecells_goal

        # variables
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10)
        # self.freecells_start = []
        # self.freecells_goal = []
        #
        self.clock = None
        self.tf = []
        self.robot_pose = None
        # self.bsf = None
        # self.bsb = None
        # self.pc = None
        #
        self.experiment_finished = False
        self.robot_updated = False
        self.status = Status.NONE
        self.factor = 0
        self.people = []
        self.nodes = rosnode.get_node_names()

        self.checkpoints = []
        self.checkpoint_actual_index = 0

        # publishers
        self.pub_initpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        # self.pub_freecells_start = rospy.Publisher('/freecells_start', MarkerArray, queue_size=10)
        # self.pub_freecells_goal = rospy.Publisher('/freecells_goal', MarkerArray, queue_size=10)

        # subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        rospy.Subscriber('/collision', String, self.collision_callback)
        rospy.Subscriber('/check_forbidden_region', Region, self.forbidden_callback)
        rospy.Subscriber('/real_time_factor', Float32, self.factor_callback)
        rospy.Subscriber('/people', People, self.people_callback)
        # rospy.Subscriber('/map', OccupancyGrid, self.map1_callback)
        # rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map2_callback)
        # rospy.Subscriber('/base_scan_front', LaserScan, self.bsf_callback)
        # rospy.Subscriber('/base_scan_back', LaserScan, self.bsb_callback)
        # rospy.Subscriber('/xtion/depth/points', PointCloud2, self.pc_callback)

        # services
        s0 = '/gazebo/reset_world'
        rospy.loginfo('Waiting for "'+ s0 +'" service')
        rospy.wait_for_service(s0)
        self.srv_reset_world = rospy.ServiceProxy(s0, Empty)
        s1 = '/gazebo/set_model_state'
        rospy.loginfo('Waiting for "'+ s1 +'" service')
        rospy.wait_for_service(s1)
        self.srv_model_reposition = rospy.ServiceProxy(s1, SetModelState)
        s2 = '/move_base/clear_costmaps'
        rospy.loginfo('Waiting for "'+ s2 +'" service')
        rospy.wait_for_service(s2)
        self.srv_clear_costmaps = rospy.ServiceProxy(s2, Empty)
        s3 = '/move_base/'+self.global_planner.split("/", 1)[1]+'/make_plan'
        rospy.loginfo('Waiting for "'+ s3 +'" service')
        rospy.wait_for_service(s3)
        self.srv_make_plan = rospy.ServiceProxy(s3, GetPlan)
        # s4 = '/regions/start'
        # rospy.loginfo('Waiting for "'+ s4 +'" service')
        # rospy.wait_for_service(s4)
        # self.srv_regions_start = rospy.ServiceProxy(s4, Regions)
        # print ('')
        # s5 = '/regions/goal'
        # rospy.loginfo('Waiting for "'+ s5 +'" service')
        # rospy.wait_for_service(s5)
        # self.srv_regions_goal = rospy.ServiceProxy(s5, Regions)
        # print ('')

        # actions
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # get freecells
        # img_freecells_start = cv2.imread(self.path_img_freecells_start,0)
        # img_freecells_goal = cv2.imread(self.path_img_freecells_goal,0)
        # (h_s, w_s) = img_freecells_start.shape[:2]
        # (h_g, w_g) = img_freecells_goal.shape[:2]
        # MS = cv2.getRotationMatrix2D((w_s/2, h_s/2), -90, 1.0)
        # MG = cv2.getRotationMatrix2D((w_g/2, h_g/2), -90, 1.0)
        # img_freecells_start = cv2.warpAffine(img_freecells_start, MS, (h_s,w_s))
        # img_freecells_goal = cv2.warpAffine(img_freecells_goal, MG, (h_g,w_g))
        # a_s = numpy.argwhere(img_freecells_start == 255)
        # a_g = numpy.argwhere(img_freecells_goal == 255)
        # self.freecells_start = a_s*0.05 - 100
        # self.freecells_goal = a_g*0.05 - 100
        # start_marker_array = self.get_freecells_markers(self.freecells_start)
        # goal_marker_array = self.get_freecells_markers(self.freecells_goal)
        # self.pub_freecells_start.publish(start_marker_array)
        # self.pub_freecells_goal.publish(goal_marker_array)

        # self.freecells_start = self.srv_regions_start().points
        # self.freecells_goal = self.srv_regions_goal().points
        # start_marker_array = self.get_freecells_markers(self.freecells_start)
        # goal_marker_array = self.get_freecells_markers(self.freecells_goal)
        # self.pub_freecells_start.publish(start_marker_array)
        # self.pub_freecells_goal.publish(goal_marker_array)

    #### calbacks ####

    def model_callback(self, data):
        try:
            index = data.name.index(self.robot_model_name)
            self.robot_pose = data.pose[index]
            self.robot_updated = True
        except ValueError as e:
            pass

    def movebase_callback(self, state, result):
        # if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
        if (state == 3):
            rospy.loginfo('reached checkpoint ' + str(self.checkpoint_actual_index))
            self.checkpoint_actual_index += 1
            if(self.checkpoint_actual_index == len(self.checkpoints)):
                self.experiment_finished = True
                self.status = Status.SUCCESS
            else:
                self.send_move_base_command(self.checkpoints[self.checkpoint_actual_index])
        else:
            self.experiment_finished = True
            self.status = Status.ABORTION

    def tf_callback(self, data):
        self.tf = data
    def clock_callback(self, data):
        self.clock = data

    def collision_callback(self, msg):
        # rospy.loginfo('Collision detected with ' + msg.data)
        self.status = Status.COLLISION
        self.experiment_finished = True

    def forbidden_callback(self, msg):
        # rospy.loginfo('forbidden region invaded: point(' + msg.x + "," + msg.y + ")" )
        self.status = Status.INVASION
        self.experiment_finished = True

    def factor_callback(self, msg):
        # rospy.loginfo('Real time factor: ' + str(msg.data))
        self.factor = msg.data

    def people_callback(self, msg):
        self.people = msg.people

    # def bsf_callback(self, data):
    #     self.bsf = data
    # def bsb_callback(self, data):
    #     self.bsb = data
    # def pc_callback(self, data):
    #     self.pc = data
    #
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

    ### functions ####

    def get_region_random(self, region_service):
        rospy.loginfo('Waiting for "'+ region_service +'" service')
        rospy.wait_for_service(region_service)
        srv_regions = rospy.ServiceProxy(region_service, Regions)
        freecells = srv_regions().points

        # get robot random start pose
        pose = Pose()
        pose_angle = random.randint(0,360)
        pose_index = random.randint(0,len(freecells)-1)
        pose_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(pose_angle))
        pose.position.x = freecells[pose_index].x
        pose.position.y = freecells[pose_index].y
        pose.position.z = 0.1
        pose.orientation.x = pose_quaternion[0]
        pose.orientation.y = pose_quaternion[1]
        pose.orientation.z = pose_quaternion[2]
        pose.orientation.w = pose_quaternion[3]
        pose_stamped = PoseStamped(Header(0,rospy.Time.now(),'map'), pose)

        return pose_stamped

    def get_checkpoints_random(self, regions_service):
        rospy.loginfo('Waiting for "'+ regions_service +'" service')
        rospy.wait_for_service(regions_service)
        srv_regions = rospy.ServiceProxy(regions_service, Regions)

        checkpoints = []
        for region in srv_regions().regions:
            # get randon point
            pose = Pose()
            pose_angle = random.randint(0,360)
            pose_index = random.randint(0,len(region.points)-1)
            pose_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(pose_angle))
            pose.position.x = region.points[pose_index].x
            pose.position.y = region.points[pose_index].y
            pose.position.z = 0.1
            pose.orientation.x = pose_quaternion[0]
            pose.orientation.y = pose_quaternion[1]
            pose.orientation.z = pose_quaternion[2]
            pose.orientation.w = pose_quaternion[3]

            checkpoints.append(PoseStamped(Header(0,rospy.Time.now(),'map'), pose))

        return checkpoints


    def get_min_dist_time(self, poses, robot_vel):

        space_min = 0
        time_min = 0

        p = Point()
        p.x = poses[0].pose.position.x
        p.y = poses[0].pose.position.y

        # set minimum space to reach a goal
        space_min += math.sqrt(pow((poses[0].pose.position.x - p.x), 2)+
                          pow((poses[0].pose.position.y - p.y), 2))
        for k in range(1,len(poses)):
            space_min += math.sqrt(
                pow((poses[k].pose.position.x - poses[k-1].pose.position.x), 2)+
                pow((poses[k].pose.position.y - poses[k-1].pose.position.y), 2))

        # set minimum time to reach a goal
        time_min = space_min/robot_vel;

        return (space_min, time_min)

    # def get_freecells_markers(self, freecells):
    #     markerArray = MarkerArray()
    #     id = 0
    #     for fc in freecells:
    #
    #         marker = Marker()
    #         marker.id = id
    #         marker.header.frame_id = "/map"
    #         marker.type = marker.SPHERE
    #         marker.action = marker.ADD
    #         marker.scale.x = 0.05
    #         marker.scale.y = 0.05
    #         marker.scale.z = 0.05
    #         marker.color.a = 1.0
    #         marker.color.r = 1.0
    #         marker.color.g = 1.0
    #         marker.color.b = 0.0
    #         marker.pose.orientation.w = 1.0
    #         marker.pose.position.x = fc.x
    #         marker.pose.position.y = fc.y
    #         marker.pose.position.z = fc.z
    #
    #         markerArray.markers.append(marker)
    #         id += 1
    #
    #     # return the MarkerArray
    #     return markerArray

    def variables_reset(self):
        self.experiment_finished = False
        self.robot_updated = False
        self.status = Status.NONE

    def find_new_path(self,start,goal):
        path_plan = Path(Header(0,rospy.Time(0),"/map"),[])
        while(len(path_plan.poses) is 0):
            if '/amcl' in self.nodes:
                self.reset_amcl(start.pose)
            self.srv_clear_costmaps()
            path_plan.poses = self.srv_make_plan(start, goal, 0.1).plan.poses
        return path_plan

    def reset_world(self):
        self.srv_reset_world()
        self.rate.sleep()

    def reset_model(self, model_name, pose = Pose()):
        model = ModelState()
        model.model_name = model_name
        model.pose = pose
        self.srv_model_reposition(model)
        self.rate.sleep()

    def reset_amcl(self, start_pose):
        # Reset robot amcl position
        initpose = PoseWithCovarianceStamped()
        initpose.header = Header(0,rospy.Time.now(),"/map")
        initpose.pose.pose = start_pose
        self.pub_initpose.publish(initpose)
        self.rate.sleep()

    def get_clock(self):
        return self.clock

    def robot_update(self, start):

        rospy.loginfo('Waiting for robot update...')

        self.robot_updated = False;
        while not rospy.is_shutdown():
            self.rate.sleep()
            if(self.robot_updated):
                break
        while self.status is not Status.NONE:
            self.variables_reset()
            self.rate.sleep()

        if '/amcl' in self.nodes:
            self.reset_amcl(start.pose)

        # clear costmaps
        self.srv_clear_costmaps()
        self.rate.sleep()

        rospy.loginfo('Robot ready.')

    def send_move_base_command(self, goal):
        # send commando to move_base
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header = Header(0,rospy.Time(0),"map")
        mb_goal.target_pose.pose = goal.pose
        self.move_base.send_goal(mb_goal, done_cb=self.movebase_callback)
        # self.move_base.wait_for_result()
        self.rate.sleep()

    def cancel_all_goals(self):
        # stop tracking end cancel finished move_base goal
        self.move_base.cancel_all_goals()
        self.move_base.stop_tracking_goal();
        self.rate.sleep()



    def start(self, data):

            # data.factor_array.append(self.factor)
            # data.people_array.append(self.people)
            # data.localization_error_array.append(0)
            # data.path_executed.append(data.start.pose.position)
            # data.delta_space.append(0)
            # data.delta_time append(rospy.Time.now())
            #
            # data.total_space = total_space
            # data.total_time = total_time
            # data.status = self.status.name



            # delta_space = []
            # total_space = 0
            # s_begin = data.start.pose.position
            # delta_space.append(0)
            # path_executed = []
            # path_executed.append(s_begin)

            # delta_time = []
            # total_time = 0
            # t_begin = rospy.Time.now()
            # delta_time.append(t_begin)

            # factor_array = []
            # factor_array.append(self.factor)

            # people_array = []
            # people_array.append(self.people)

            # localization_error_array = []
            # localization_error_array.append(0)

            self.checkpoints = data.checkpoints
            self.checkpoint_actual_index = 1

            # loop
            step = 0
            while not rospy.is_shutdown():
                self.rate.sleep()

                # factor_array.append(self.factor)
                # people_array.append(self.people)

		        # localization error
            	(trans,rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                error = math.sqrt(pow(trans[0], 2) + pow(trans[1], 2))
                # localization_error_array.append(error)

                # update space
                s_0 = data.path_executed[-1]
                s_1 = self.robot_pose.position
                delta_space = math.sqrt(
                    pow((s_1.x - s_0.x), 2)+
                    pow((s_1.y - s_0.y), 2))

                # delta_space.append(s_delta)
                # total_space += delta_space[-1]
                # s_begin = s_now;
                # path_executed.append(s_begin)

                # update time
                t_0 = data.delta_time[0]
                t_1 = rospy.Time.now()
                delta_time = (t_1 - t_0)
                # total_time += delta_time.to_sec()
                # t_begin = t_now;

                data.factor_array.append(self.factor)
                data.people_array.append(self.people)
                data.localization_error_array.append(error)
                data.path_executed.append(s_1)

                data.delta_space.append(delta_space)
                data.total_space += delta_space

                data.delta_time.append(delta_time)
                data.total_time = delta_time.to_sec()

                # check space restriction
                if(data.total_space > data.space_max):
                    self.experiment_finished = True
                    self.status = Status.SPACE_EXCEEDED

                # check time restriction
                if(data.total_time > data.time_max):
                    self.experiment_finished = True
                    self.status = Status.TIME_EXCEEDED


                # break
                if(self.experiment_finished):
                     break

                # step increment
                step = step+1

            # #
            # data.factor_array = factor_array
            # data.people_array = people_array
            # data.localization_error_array = localization_error_array
            # data.path_executed = path_executed
            # data.delta_space = delta_space
            # data.delta_time = delta_time
            # data.total_space = total_space
            # data.total_time = total_time
            data.status = self.status.name
