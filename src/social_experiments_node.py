#!/usr/bin/env python

import tf
import rospy
# import rosbag

from experiments import Experiments, Data

class SocialExperimentsNode():

    def __init__(self):

        # parameters
        self.world_model_name = rospy.get_param('social_experiments/world_model_name', '')
        self.robot_model_name = rospy.get_param('social_experiments/robot_model_name', '')
        self.max_experiments = rospy.get_param('social_experiments/max_experiments', 100)
        self.path_storage = rospy.get_param('social_experiments/path_storage', '')
        self.robot_vel = rospy.get_param('social_experiments/robot_vel', 0.3)
        self.space_factor_tolerance = rospy.get_param('social_experiments/space_factor_tolerance', 5)
        self.time_factor_tolerance = rospy.get_param('social_experiments/time_factor_tolerance', 5)
        self.path_img_freecells_start = rospy.get_param('social_experiments/path_img_freecells_start', '')
        self.path_img_freecells_goal = rospy.get_param('social_experiments/path_img_freecells_goal', '')

        # log
        rospy.loginfo('world_model_name: ' + self.world_model_name)
        rospy.loginfo('robot: ' + self.robot_model_name)
        rospy.loginfo('robot vel: ' + str(self.robot_vel))
        rospy.loginfo('space factor tolerance: ' + str(self.space_factor_tolerance))
        rospy.loginfo('time factor tolerance: ' + str(self.time_factor_tolerance))
        rospy.loginfo('max experiments: ' + str(self.max_experiments))
        print ('')

        # # bag
        # self.bag = None
        self.data = []

        # init experiments
        self.ex = Experiments(self.world_model_name, self.robot_model_name,
            self.path_img_freecells_start, self.path_img_freecells_goal)

    def start_experiments(self):
        # experiments loop
        for i in range(0, self.max_experiments):
            rospy.loginfo('Preparing experiment %i/%i' % (i+1, self.max_experiments))
            self.data.append(Data())

            # # open bag
            # self.bag = rosbag.Bag(self.path_bags_storage + "/" + str(i)+'.bag', 'w')
            # self.bag.write('/clock', self.ex.get_clock())

            rospy.loginfo('Fiding new start and goal poses...')
            (self.data[-1].start, self.data[-1].goal) = self.ex.get_start_and_goal_random()
            # self.bag.write('/start', self.data[-1].start)
            # self.bag.write('/goal', self.data[-1].goal)
            rospy.loginfo('Start'
            + '(x=' + str(self.data[-1].start.pose.position.x)
            + ',y=' + str(self.data[-1].start.pose.position.x)
            + ',ang=' + str(self.data[-1].start.pose.orientation.z) + ')')
            rospy.loginfo('Goal'
            + '(x=' + str(self.data[-1].goal.pose.position.x)
            + ',y=' + str(self.data[-1].goal.pose.position.y)
            + ',ang=' + str(self.data[-1].goal.pose.orientation.z) + ')')

            rospy.loginfo('Finding a path plan...')
            self.data[-1].path_plan = self.ex.find_new_path(self.data[-1].start,self.data[-1].goal)
            # self.bag.write('/path_plan', self.data[-1].path_plan)
            rospy.loginfo('Path plan size: ' + str(len(self.data[-1].path_plan.poses)))

            rospy.loginfo('Resetting world model')
            self.ex.reset_model(self.world_model_name)

            rospy.loginfo('Resetting robot model')
            self.ex.reset_model(self.robot_model_name, self.data[-1].start.pose)

            rospy.loginfo("setting min dist and time to reach destination")
            (self.data[-1].space_min, self.data[-1].time_min) = self.ex.get_min_dist_time(self.data[-1].path_plan.poses, self.robot_vel)
            rospy.loginfo('Space min: ' + str(self.data[-1].space_min) + ' meters')
            rospy.loginfo('Time min: ' + str(self.data[-1].time_min) + ' seconds')

            rospy.loginfo("setting max dist and time to reach destination")
            self.data[-1].space_max = self.data[-1].space_min*self.space_factor_tolerance
            self.data[-1].time_max = self.data[-1].time_min*self.time_factor_tolerance
            rospy.loginfo('Space max: ' + str(self.data[-1].space_max) + ' meters')
            rospy.loginfo('Time max: ' + str(self.data[-1].time_max) + ' seconds')


            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))
            self.ex.start(self.data[-1])


            rospy.loginfo('Space elapsed: ' + str(self.data[-1].total_space) + ' meters')
            rospy.loginfo('Time elapsed: ' + str(self.data[-1].total_time) + ' seconds')
            rospy.loginfo('Status: ' + self.data[-1].status)
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments))
            print ('')

            # # close bag
            # self.bag.close()

    def generate_csv(self):
        # print params
        file_params = open(self.path_storage+"/params.yaml","w+")
        file_params.write("environment: " + str(self.world_model_name) + "\n")
        file_params.write("robot_name: " + str(self.robot_model_name) + "\n")
        file_params.write("robot_vel: " + str(self.robot_vel) + "\n")
        file_params.write("space_factor_tolerance: " + str(self.space_factor_tolerance) + "\n")
        file_params.write("time_factor_tolerance: " + str(self.time_factor_tolerance) + "\n")
        file_params.write("max_experiments: " + str(self.max_experiments) + "\n")
        file_params.close()


        # print real time factor
        file_factor = open(self.path_storage+"/real_time_factor.json","w+")
        i = 0
        list_e = []
        for e1 in self.data:
            list_e.append('"'+str(i)+'":[' + ','.join([str(x) for x in e1.factor_array]) + ']')
            i += 1
        file_factor.write('{'+ ',\n'.join([str(x) for x in list_e]) +'}')
        file_factor.close()


        # print path plan
        file_path_min_x = open(self.path_storage+"/path_plan_x.json","w+")
        file_path_min_y = open(self.path_storage+"/path_plan_y.json","w+")
        i = 0
        list_ex = []
        list_ey = []
        for e1 in self.data:
            list_x = []
            list_y = []
            for e2 in e1.path_plan.poses:
                list_x.append(e2.pose.position.x)
                list_y.append(e2.pose.position.y)
            list_ex.append('"'+str(i)+'":[' + ','.join([str(x) for x in list_x]) + ']')
            list_ey.append('"'+str(i)+'":[' + ','.join([str(y) for y in list_y]) + ']')
            i += 1
        file_path_min_x.write('{'+ ',\n'.join([str(x) for x in list_ex]) +'}')
        file_path_min_y.write('{'+ ',\n'.join([str(y) for y in list_ey]) +'}')
        file_path_min_x.close()
        file_path_min_y.close()

        # print path executed
        file_path_elapsed_x = open(self.path_storage+"/path_executed_x.json","w+")
        file_path_elapsed_y = open(self.path_storage+"/path_executed_y.json","w+")
        i = 0
        list_ex = []
        list_ey = []
        for e1 in self.data:
            list_x = []
            list_y = []
            for e2 in e1.path_executed:
                list_x.append(e2.x)
                list_y.append(e2.y)
            list_ex.append('"'+str(i)+'":[' + ','.join([str(x) for x in list_x]) + ']')
            list_ey.append('"'+str(i)+'":[' + ','.join([str(y) for y in list_y]) + ']')
            i += 1
        file_path_elapsed_x.write('{'+ ',\n'.join([str(x) for x in list_ex]) +'}')
        file_path_elapsed_y.write('{'+ ',\n'.join([str(y) for y in list_ey]) +'}')
        file_path_elapsed_x.close()
        file_path_elapsed_y.close()

        # print result
        file_result = open(self.path_storage+"/result.csv","w+")
        file_result.write("i,start_x,start_y,start_ang,goal_x,goal_y,goal_ang," +
                        "space_min,time_min,space_elapsed,time_elapsed,status\n")
        i = 0
        for e1 in self.data:
            (_, _, start_yaw) = tf.transformations.euler_from_quaternion(
                [e1.start.pose.orientation.x, e1.start.pose.orientation.y,
                e1.start.pose.orientation.z, e1.start.pose.orientation.w])
            (_, _, goal_yaw) = tf.transformations.euler_from_quaternion(
                [e1.goal.pose.orientation.x, e1.goal.pose.orientation.y,
                e1.goal.pose.orientation.z, e1.goal.pose.orientation.w])
            file_result.write( str(i) + ",")
            file_result.write( str(e1.start.pose.position.x) + ",")
            file_result.write( str(e1.start.pose.position.y) + ",")
            file_result.write( str(start_yaw) + ",")
            file_result.write( str(e1.goal.pose.position.x) + ",")
            file_result.write( str(e1.goal.pose.position.y) + ",")
            file_result.write( str(goal_yaw) + ",")
            file_result.write( str(e1.space_min) + ",")
            file_result.write( str(e1.time_min) + ",")
            file_result.write( str(e1.total_space) + ",")
            file_result.write( str(e1.total_time) + ",")
            file_result.write( str(e1.status) + "\n")
            i += 1
        file_result.close()


if __name__ == '__main__':
    rospy.init_node('social_experiments_node')

    sen = SocialExperimentsNode()
    sen.start_experiments()
    sen.generate_csv()
