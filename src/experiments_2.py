#!/usr/bin/env python

import tf
import cv2
import math
import time
import rospy
import numpy
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
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from gazebo_msgs.srv import SetModelState
from util_ros.srv import PointClear
from nav_msgs.srv import GetPlan
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
        self.start_x = None
        self.start_y = None
        self.start_ang = None
        self.goal_x = None
        self.goal_y = None
        self.goal_ang = None
        self.path_min = []
        self.space_min = None
        self.time_min = None

        # pos experiment info
        self.status = None
        self.v_localization_error = []
        self.v_space_elapsed = []
        self.v_time_elapsed = []
        self.space_elapsed = None
        self.time_elapsed = None

class Experiments():
    """docstring for Experiments"""
    def __init__(self):
        rospy.init_node('Experiments')
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.experiments = []
        self.freecells = []
        self.robot_position = None
        self.start_position = None
        self.goal_position = None
        self.start_quaternion = None
        self.goal_quaternion = None
        # self.map1 = None
        # self.map2 = None

        # parameters
        self.environment = rospy.get_param('phd_experiments/environment', '')
        self.robot_name = rospy.get_param('phd_experiments/robot_name', '')
        self.robot_vel = rospy.get_param('phd_experiments/robot_vel', 0.3)
        self.xy_goal_tolerance = rospy.get_param('phd_experiments/xy_goal_tolerance', 0.1)
        self.space_factor_tolerance = rospy.get_param('phd_experiments/space_factor_tolerance', 5)
        self.time_factor_tolerance = rospy.get_param('phd_experiments/time_factor_tolerance', 5)
        self.max_experiments = rospy.get_param('phd_experiments/max_experiments', 100)
        self.path_img_freecells = rospy.get_param('phd_experiments/path_img_freecells', '')

        # publishers
        self.pub_initpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.publ_freecells = rospy.Publisher('/freecells', MarkerArray, queue_size=10)

        # subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map1_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map2_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        rospy.Subscriber('/collision', Bool, self.collision_callback)

        # services
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_reposition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        self.make_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        rospy.wait_for_service('/util_move_base/point_clear_global')
        self.srv_point_clear = rospy.ServiceProxy('/util_move_base/point_clear_global', PointClear)

        # actions
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

    #### calbacks ####

    def map1_callback(self, data):
        # self.map1 = data.data
        img = Image.new('RGB', (data.info.width, data.info.height))
        img.putdata(data.data)
        img.save('map.png')

    def map2_callback(self, data):
        # self.map2 = data.data
        img = Image.new('RGB', (data.info.width, data.info.height))
        img.putdata(data.data)
        img.save('costmap.png')



    def model_callback(self, data):
        try:
            index = data.name.index(self.robot_name)
            self.robot_position = data.pose[index].position
            self.robot_updated = True
        except ValueError as e:
            pass

    def collision_callback(self, data):
        if (data.data == True):
            self.experiment_finished = True
            self.status = Status.COLLISION

    def movebase_callback(self, state, result):
        self.experiment_finished = True
        # if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
        if (state == 3):
            self.status = Status.SUCCESS
        else:
            self.status = Status.ABORTION


    #### functions ####

    def set_start_and_goal_random(self, data):

        # get robot random start position
        start_angle = random.randint(0,360)
        self.start_position = random.randint(0,len(self.freecells))
        self.start_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(start_angle))
        data.start_x = self.freecells[self.start_position][0]
        data.start_y = self.freecells[self.start_position][1]
        data.start_ang = math.radians(start_angle)

        # get robot random goal position
        goal_angle = random.randint(0,360)
        self.goal_position = random.randint(0,len(self.freecells))
        self.goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(goal_angle))
        data.goal_x = self.freecells[self.goal_position][0]
        data.goal_y = self.freecells[self.goal_position][1]
        data.goal_ang = math.radians(goal_angle)

    def get_plan(self):
        plan = []
        while(len(plan) is 0):
            self.rate.sleep()
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = 'map'
            start.pose.position.x = self.freecells[self.start_position][0]
            start.pose.position.y = self.freecells[self.start_position][1]
            start.pose.orientation.x = self.start_quaternion[0]
            start.pose.orientation.y = self.start_quaternion[1]
            start.pose.orientation.z = self.start_quaternion[2]
            start.pose.orientation.w = self.start_quaternion[3]
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            goal.pose.position.x = self.freecells[self.goal_position][0]
            goal.pose.position.y = self.freecells[self.goal_position][1]
            goal.pose.orientation.x = self.goal_quaternion[0]
            goal.pose.orientation.y = self.goal_quaternion[1]
            goal.pose.orientation.z = self.goal_quaternion[2]
            goal.pose.orientation.w = self.goal_quaternion[3]
            tolerance = 0.0
            plan = self.make_plan(start, goal, tolerance).plan.poses

        return plan

    def set_min_dist_time(self, plan, data):
        space_min = 0
        p = Point()
        p.x = data.start_x
        p.y = data.start_y
        data.path_min.append(p)
        data.path_min.append(plan[0].pose.position)
        space_min += math.sqrt(pow((plan[0].pose.position.x - p.x), 2)+
                          pow((plan[0].pose.position.y - p.y), 2))
        for k in range(1,len(plan)):
            space_min += math.sqrt(pow((plan[k].pose.position.x
                                 - plan[k-1].pose.position.x), 2)+
                              pow((plan[k].pose.position.y
                                 - plan[k-1].pose.position.y), 2))
            data.path_min.append(plan[k].pose.position)

        # set minimum space and time to reach a goal
        data.space_min = space_min;
        data.time_min = space_min/self.robot_vel;

    def pub_freecells_markers(self):
        markerArray = MarkerArray()
        id = 0
        for fc in self.freecells:

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

        # Publish the MarkerArray
        self.publ_freecells.publish(markerArray)


    def start(self):
        rospy.loginfo('environment: ' + self.environment)
        rospy.loginfo('robot: ' + self.robot_name)
        rospy.loginfo('robot vel: ' + str(self.robot_vel))
        rospy.loginfo('space factor tolerance: ' + str(self.space_factor_tolerance))
        rospy.loginfo('time factor tolerance: ' + str(self.time_factor_tolerance))
        rospy.loginfo('max experiments: ' + str(self.max_experiments))

        # get freecells
        img_freecells = cv2.imread(self.path_img_freecells,0)
        (h, w) = img_freecells.shape[:2]
        M = cv2.getRotationMatrix2D((w/2, h/2), -90, 1.0)
        img_freecells = cv2.warpAffine(img_freecells, M, (h,w))
        a = numpy.argwhere(img_freecells == 255)
        self.freecells = a*0.05 - 100
        self.pub_freecells_markers()

        # experiments loop
        for i in range(0, self.max_experiments):

            # store experiments
            data = Data()

            # reset variables
            self.experiment_finished = False
            self.status = Status.NONE

            # set start and goal random
            self.set_start_and_goal_random(data)

            # get plan
            rospy.loginfo('Finding a plan...')
            plan_is_clear = False
            count = 0
            while not plan_is_clear:
                count += 1
                if(count == 100):
                    self.set_start_and_goal_random(data)
                    count = 0

                plan_is_clear = True
                plan = self.get_plan()
                for p in plan:
                    point_is_clear = self.srv_point_clear(p.pose.position).response
                    if not point_is_clear:
                        plan_is_clear = False


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
            ms_robot.pose.position.x = self.freecells[self.start_position][0]
            ms_robot.pose.position.y = self.freecells[self.start_position][1]
            ms_robot.pose.position.z = 0.1
            ms_robot.pose.orientation.x = self.start_quaternion[0]
            ms_robot.pose.orientation.y = self.start_quaternion[1]
            ms_robot.pose.orientation.z = self.start_quaternion[2]
            ms_robot.pose.orientation.w = self.start_quaternion[3]
            self.model_reposition(ms_robot)
            self.rate.sleep()

            # Reset robot amcl position
            initpose = PoseWithCovarianceStamped()
            initpose.header.stamp = rospy.Time.now()
            initpose.header.frame_id = 'map'
            initpose.pose.pose.position.x = self.freecells[self.start_position][0]
            initpose.pose.pose.position.y = self.freecells[self.start_position][1]
            initpose.pose.pose.orientation.x = self.start_quaternion[0]
            initpose.pose.pose.orientation.y = self.start_quaternion[1]
            initpose.pose.pose.orientation.z = self.start_quaternion[2]
            initpose.pose.pose.orientation.w = self.start_quaternion[3]
            self.pub_initpose.publish(initpose)
            self.rate.sleep()

            # clear costmaps
            self.clear_costmaps()
            self.rate.sleep()

            # set min dist and time to reach destination
            self.set_min_dist_time(plan, data)

            rospy.loginfo('Waiting for robot position update...')
            self.robot_updated = False;
            while not rospy.is_shutdown():
                self.rate.sleep()
                if(self.robot_updated):
                    break


            # start
            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))

            # print pre info
            rospy.loginfo('PreInfo: start'
            + '(x=' + str(data.start_x)
            + ',y=' + str(data.start_y)
            + ',ang=' + str(data.start_ang) + ')')
            rospy.loginfo('PreInfo: goal'
            + '(x=' + str(data.goal_x)
            + ',y=' + str(data.goal_y)
            + ',ang=' + str(data.goal_ang) + ')')
            rospy.loginfo('PreInfo: Space min: ' + str(data.space_min) + ' meters')
            rospy.loginfo('PreInfo: Time min: ' + str(data.time_min) + ' seconds')

            # send commando to move_base
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.freecells[self.goal_position][0]
            goal.target_pose.pose.position.y = self.freecells[self.goal_position][1]
            goal.target_pose.pose.orientation.x = self.goal_quaternion[0]
            goal.target_pose.pose.orientation.y = self.goal_quaternion[1]
            goal.target_pose.pose.orientation.z = self.goal_quaternion[2]
            goal.target_pose.pose.orientation.w = self.goal_quaternion[3]
            self.move_base.send_goal(goal, done_cb=self.movebase_callback)
            # self.move_base.wait_for_result()

            #
            space_elapsed = 0
            s_begin = Point()
            s_begin.x = data.start_x
            s_begin.y = data.start_y
            s_begin.z = 0.0
            data.v_space_elapsed.append(s_begin)

            #
            time_elapsed = 0
            t_begin = time.time()
            data.v_time_elapsed.append(t_begin)

            # loop
            while not rospy.is_shutdown():
                self.rate.sleep()

		        # update localization error
            	(trans,rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                error = math.sqrt(pow(trans[0], 2) + pow(trans[1], 2))
                data.v_localization_error.append(error)

                # update space
                s_now = self.robot_position
                space_elapsed += math.sqrt(pow((s_now.x - s_begin.x), 2)+
                                           pow((s_now.y - s_begin.y), 2))
                s_begin = s_now;
                data.v_space_elapsed.append(s_begin)

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

            # set pos info
            data.space_elapsed = space_elapsed
            data.time_elapsed = time_elapsed
            data.status = self.status

            # print pos info
            rospy.loginfo('PosInfo: Space elapsed: ' + str(data.space_elapsed) + ' meters')
            rospy.loginfo('PosInfo: Time elapsed: ' + str(data.time_elapsed) + ' seconds')
            rospy.loginfo('PosInfo: Status: ' + data.status.name)

            # finish
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments))

            # store experiment
            self.experiments.append(data)

            # stop tracking end cancel finished move_base goal
            self.move_base.cancel_all_goals()
            self.move_base.stop_tracking_goal();


    def result(self):

      success = 0
      mean_space_coef = 0
      mean_time_coef = 0
      sd_space_coef = 0
      sd_time_coef = 0

      failure_by_space_exceeded = 0
      failure_by_time_exceeded = 0
      failure_by_abortion = 0
      failure_by_collision = 0

      # print params in cvs file
      file_params = open("params.yaml","w+")
      file_params.write("environment: " + str(self.environment) + "\n")
      file_params.write("robot_name: " + str(self.robot_name) + "\n")
      file_params.write("robot_vel: " + str(self.robot_vel) + "\n")
      file_params.write("space_factor_tolerance: " + str(self.space_factor_tolerance) + "\n")
      file_params.write("time_factor_tolerance: " + str(self.time_factor_tolerance) + "\n")
      file_params.write("max_experiments: " + str(self.max_experiments) + "\n")
      file_params.close()


      # print localization error in cvs file
      file_localization_error = open("localization_error.json","w+")
      i = 0
      list_ele = []
      for e1 in self.experiments:
          list_ele.append('"'+str(i)+'":[' + ','.join([str(x) for x in e1.v_localization_error]) + ']')
          i += 1
      file_localization_error.write('{'+ ',\n'.join([str(x) for x in list_ele]) +'}')
      file_localization_error.close()

      # print path min in cvs file
      file_path_min_x = open("path_min_x.json","w+")
      file_path_min_y = open("path_min_y.json","w+")
      i = 0
      list_ex = []
      list_ey = []
      for e1 in self.experiments:
          list_x = []
          list_y = []
          for e2 in e1.path_min:
              list_x.append(e2.x)
              list_y.append(e2.y)
          list_ex.append('"'+str(i)+'":[' + ','.join([str(x) for x in list_x]) + ']')
          list_ey.append('"'+str(i)+'":[' + ','.join([str(y) for y in list_y]) + ']')
          i += 1
      file_path_min_x.write('{'+ ',\n'.join([str(x) for x in list_ex]) +'}')
      file_path_min_y.write('{'+ ',\n'.join([str(y) for y in list_ey]) +'}')
      file_path_min_x.close()
      file_path_min_y.close()

      # print path elapsed in cvs file
      file_path_elapsed_x = open("path_elapsed_x.json","w+")
      file_path_elapsed_y = open("path_elapsed_y.json","w+")
      i = 0
      list_ex = []
      list_ey = []
      for e1 in self.experiments:
          list_vx = []
          list_vy = []
          for e2 in e1.v_space_elapsed:
              list_vx.append(e2.x)
              list_vy.append(e2.y)
          list_ex.append('"'+str(i)+'":[' + ','.join([str(x) for x in list_vx]) + ']')
          list_ey.append('"'+str(i)+'":[' + ','.join([str(y) for y in list_vy]) + ']')
          i += 1
      file_path_elapsed_x.write('{'+ ',\n'.join([str(x) for x in list_ex]) +'}')
      file_path_elapsed_y.write('{'+ ',\n'.join([str(y) for y in list_ey]) +'}')
      file_path_elapsed_x.close()
      file_path_elapsed_y.close()

      # print result in cvs file
      file_result = open("result.csv","w+")
      file_result.write("i,start_x,start_y,start_ang,goal_x,goal_y,goal_ang," +
                        "space_min,time_min,space_elapsed,time_elapsed,status\n")
      i = 0
      for e1 in self.experiments:
          file_result.write( str(i+1) + ",")
          file_result.write( str(e1.start_x) + ",")
          file_result.write( str(e1.start_y) + ",")
          file_result.write( str(e1.start_ang) + ",")
          file_result.write( str(e1.goal_x) + ",")
          file_result.write( str(e1.goal_y) + ",")
          file_result.write( str(e1.goal_ang) + ",")
          file_result.write( str(e1.space_min) + ",")
          file_result.write( str(e1.time_min) + ",")
          file_result.write( str(e1.space_elapsed) + ",")
          file_result.write( str(e1.time_elapsed) + ",")
          file_result.write( str(e1.status.name) + "\n")
          i += 1
      file_result.close()


if __name__ == '__main__':
    ex = Experiments()
    ex.start()
    ex.result()
