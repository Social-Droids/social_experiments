#!/usr/bin/env python

import tf
import rospy
# import rosbag

from experiments import Experiments, Data

class SocialExperimentsNode():

    def __init__(self):

        # parameters
        self.global_planner = rospy.get_param('social_experiments/global_planner', '')
        self.local_planner = rospy.get_param('social_experiments/local_planner', '')
        self.world_model_name = rospy.get_param('social_experiments/world_model_name', '')
        self.robot_model_name = rospy.get_param('social_experiments/robot_model_name', '')
        self.max_experiments = rospy.get_param('social_experiments/max_experiments', 100)
        self.path_storage = rospy.get_param('social_experiments/path_storage', '')
        self.robot_vel = rospy.get_param('social_experiments/robot_vel', 0.3)
        self.space_factor_tolerance = rospy.get_param('social_experiments/space_factor_tolerance', 5)
        self.time_factor_tolerance = rospy.get_param('social_experiments/time_factor_tolerance', 5)
        # self.start_service = rospy.get_param('social_experiments/start_service', '/regions/start')
        # self.goal_service = rospy.get_param('social_experiments/goal_service', '/regions/goal')
        self.checkpoint_services = rospy.get_param('social_experiments/checkpoint_services', '')

        if(self.checkpoint_services is ''):
            self.checkpoint_services = []
        else:
            self.checkpoint_services = list(self.checkpoint_services.split(" "))

        # log
        rospy.loginfo('global_planner: ' + self.global_planner)
        rospy.loginfo('local_planner: ' + self.local_planner)
        rospy.loginfo('world_model_name: ' + self.world_model_name)
        rospy.loginfo('robot: ' + self.robot_model_name)
        rospy.loginfo('robot vel: ' + str(self.robot_vel))
        rospy.loginfo('space factor tolerance: ' + str(self.space_factor_tolerance))
        rospy.loginfo('time factor tolerance: ' + str(self.time_factor_tolerance))
        rospy.loginfo('max experiments: ' + str(self.max_experiments))
        # rospy.loginfo('start service: ' + str(self.start_service))
        # rospy.loginfo('goal service: ' + str(self.goal_service))
        # rospy.loginfo('checkpoint services: ' + str(self.checkpoint_services))
        print ('')

        # data
        self.data = []

        # init experiments
        self.ex = Experiments(
            self.global_planner, self.local_planner,
            self.world_model_name, self.robot_model_name
            )

    def start_experiments(self):
        # experiments loop
        for i in range(0, self.max_experiments):
            rospy.loginfo('Preparing experiment %i/%i' % (i+1, self.max_experiments))
            self.data.append(Data())

            rospy.loginfo('Fiding checkpoints...')
            self.data[-1].checkpoints = self.ex.get_checkpoints_random("/regions/path")
            self.data[-1].path_executed.append(self.data[-1].checkpoints[0].pose.position)
            for n, cp in enumerate(self.data[-1].checkpoints):
                rospy.loginfo('checkpoint ' + str(n) + ': '
                + '(x=' + str(cp.pose.position.x)
                + ',y=' + str(cp.pose.position.x)
                + ',ang=' + str(cp.pose.orientation.z) + ')')

            rospy.loginfo('Finding a path plan...')
            for n in range(1,len(self.data[-1].checkpoints)):
                plan = self.ex.find_new_path(
                    self.data[-1].checkpoints[n-1],
                    self.data[-1].checkpoints[n]).poses
                rospy.loginfo('Path plan from checkpoint ' + str(n-1) +' to '+ str(n) +': ' +
                    str(len(plan)))
                self.data[-1].path_plan += plan
            rospy.loginfo('Total path plan size: ' + str(len(self.data[-1].path_plan)))

            self.ex.reset_world()
            rospy.loginfo('Resetting world model')
            self.ex.reset_model(self.world_model_name)
            rospy.loginfo('Resetting robot model')
            self.ex.reset_model(self.robot_model_name, self.data[-1].checkpoints[0].pose)

            rospy.loginfo("setting min dist and time to reach destination")
            (self.data[-1].space_min, self.data[-1].time_min) = self.ex.get_min_dist_time(self.data[-1].path_plan, self.robot_vel)
            rospy.loginfo('Space min: ' + str(self.data[-1].space_min) + ' meters')
            rospy.loginfo('Time min: ' + str(self.data[-1].time_min) + ' seconds')

            rospy.loginfo("setting max dist and time to reach destination")
            self.data[-1].space_max = self.data[-1].space_min*self.space_factor_tolerance
            self.data[-1].time_max = self.data[-1].time_min*self.time_factor_tolerance
            rospy.loginfo('Space max: ' + str(self.data[-1].space_max) + ' meters')
            rospy.loginfo('Time max: ' + str(self.data[-1].time_max) + ' seconds')

            self.ex.robot_update(self.data[-1].checkpoints[0])
            rospy.loginfo('Start experiment %i/%i' % (i+1, self.max_experiments))
            self.ex.send_move_base_command(self.data[-1].checkpoints[1])
            rospy.loginfo('Experiment in progress...')

            self.data[-1].delta_space.append(0)
            self.data[-1].delta_time.append(rospy.Time.now())
            self.data[-1].total_space = 0
            self.data[-1].total_time = 0

            self.ex.start(self.data[-1])
            self.ex.cancel_all_goals()

            rospy.loginfo('Space elapsed: ' + str(self.data[-1].total_space) + ' meters')
            rospy.loginfo('Time elapsed: ' + str(self.data[-1].total_time) + ' seconds')
            rospy.loginfo('Status: ' + self.data[-1].status)
            rospy.loginfo('Finish experiment ' + str(i+1) + '/' + str(self.max_experiments))
            print ('')


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
        list_f = []
        for e1 in self.data:
            list_f.append('"'+str(i)+'":[' + ','.join([str(x) for x in e1.factor_array]) + ']')
            i += 1
        file_factor.write('{'+ ',\n'.join([str(x) for x in list_f]) +'}')
        file_factor.close()

        # print localization error
        file_loc_err = open(self.path_storage+"/localization_error.json","w+")
        i = 0
        list_e = []
        for e1 in self.data:
            list_e.append('"'+str(i)+'":[' + ','.join([str(x) for x in e1.localization_error_array]) + ']')
            i += 1
        file_loc_err.write('{'+ ',\n'.join([str(x) for x in list_e]) +'}')
        file_loc_err.close()


        # print path plan
        file_path_min_x = open(self.path_storage+"/path_plan_x.json","w+")
        file_path_min_y = open(self.path_storage+"/path_plan_y.json","w+")
        i = 0
        list_ex = []
        list_ey = []
        for e1 in self.data:
            list_x = []
            list_y = []
            for e2 in e1.path_plan:
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

        # print people
        file_people = open(self.path_storage+"/people.json","w+")
        i = 0
        list_1 = []
        for e1 in self.data:
            list_2 = []
            for e2 in e1.people_array:
                list_3 = []
                for e3 in e2:
                    list_3.append('['+str(e3.position.x)+','+str(e3.position.y)+']')
                list_2.append('[' + ','.join([str(x) for x in list_3]) + ']')
            list_1.append('"'+str(i)+'":[' + ','.join([str(x) for x in list_2]) + ']')
            i += 1
        file_people.write('{'+ ',\n'.join([str(x) for x in list_1]) +'}')
        file_people.close()

        # print result
        file_result = open(self.path_storage+"/result.csv","w+")
        file_result.write("i,start_x,start_y,start_ang,goal_x,goal_y,goal_ang," +
                        "space_min,time_min,space_elapsed,time_elapsed,status\n")
        i = 0
        for e1 in self.data:
            (_, _, start_yaw) = tf.transformations.euler_from_quaternion(
                [e1.checkpoints[0].pose.orientation.x, e1.checkpoints[0].pose.orientation.y,
                e1.checkpoints[0].pose.orientation.z, e1.checkpoints[0].pose.orientation.w])
            (_, _, goal_yaw) = tf.transformations.euler_from_quaternion(
                [e1.checkpoints[-1].pose.orientation.x, e1.checkpoints[-1].pose.orientation.y,
                e1.checkpoints[-1].pose.orientation.z, e1.checkpoints[-1].pose.orientation.w])
            file_result.write( str(i) + ",")
            file_result.write( str(e1.checkpoints[0].pose.position.x) + ",")
            file_result.write( str(e1.checkpoints[0].pose.position.y) + ",")
            file_result.write( str(start_yaw) + ",")
            file_result.write( str(e1.checkpoints[-1].pose.position.x) + ",")
            file_result.write( str(e1.checkpoints[-1].pose.position.y) + ",")
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
