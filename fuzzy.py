#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import sys

front_left_sensor_reading = 0.5
front_center_sensor_reading = 0.5
front_right_sensor_reading = 0.5
right_front_sensor_reading = 0.5
right_back_sensor_reading = 0.5

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # creates publisher class

# function to control velocity of robot
def forwards(speed, turn):
	print('speed_forwards: ',speed)
	print('turn_forwards: ', turn)
	global pub
	vel_x = Twist() # initiates twist message
	vel_x.linear.x = speed  # sets linear speed of robot
	vel_x.angular.z = -turn*1.0  # sets angle to turn to pid function output (turns left?)
	pub.publish(vel_x) #publishes velocity

def callback(msg):
	global front_left_sensor_reading
	global front_center_sensor_reading
	global front_right_sensor_reading
	global right_front_sensor_reading
	global right_back_sensor_reading

	# GET THE RANGES VALUE YOU WANT
	# important info about the laser readings 
	# 	540 element is reading to the right 
	# 	630 element is reading to the front right 
	#	450 element is reading to the back right
	front_left_sensor_reading = min(msg.ranges[170:178])
	front_center_sensor_reading = min(msg.ranges[0:10])
	front_right_sensor_reading = min(msg.ranges[700:710])
	right_front_sensor_reading = min(msg.ranges[620:630])
	right_back_sensor_reading = min(msg.ranges[450:460])
	
	if math.isinf(front_left_sensor_reading) or front_left_sensor_reading > 6:
		front_left_sensor_reading = 5.9
	if math.isinf(front_center_sensor_reading) or front_center_sensor_reading > 6:
		front_center_sensor_reading = 5.9
	if math.isinf(front_right_sensor_reading) or front_right_sensor_reading > 6:
		front_right_sensor_reading = 5.9
	if math.isinf(right_front_sensor_reading) or right_front_sensor_reading > 6:
		right_front_sensor_reading = 5.9
	if math.isinf(right_back_sensor_reading) or right_back_sensor_reading > 6:
		right_back_sensor_reading = 5.9
	print('========================')
	print('front_left_sensor_reading', front_left_sensor_reading)
	print('front_center: ',front_center_sensor_reading)
	print('front right: ',front_right_sensor_reading)
	print('right_front ', right_front_sensor_reading)
	print('right back ', right_back_sensor_reading)
	print('========================')
	
def controller(action):
    global front_left_sensor_reading
    global front_center_sensor_reading
    global front_right_sensor_reading
    global right_front_sensor_reading
    global right_back_sensor_reading
    rate = rospy.Rate(25)  # sleep in loop at rate 25hz
	base_speed = 0.3 # YOU CAN CHANGE THE BASE SPEED IF YOU WANT
	while not rospy.is_shutdown():
		control_obj = Controller()
        	control_obj.mfv_dict_calc(front_left_sensor_reading, front_center_sensor_reading,
            		front_right_sensor_reading, right_front_sensor_reading, right_back_sensor_reading)
		result = control_obj.calculate_label_value(action)
		forwards(result[0], result[1])
		rate.sleep()  # pauses rate

class MembershipFunction:
    def __init__(self, mfpoint, label):
        self.points = mfpoint
        self.labels = label
        self.center = (self.points[0] + self.points[3]) / 2

    def membership_function_calc(self, x):
        if self.points[0] <= x < self.points[1]:
            return (x - self.points[0]) / (self.points[1] - self.points[0])
        elif self.points[1] <= x <= self.points[2]:
            return 1.0
        elif self.points[2] < x <= self.points[3]:
            return (self.points[3] - x) / (self.points[3] - self.points[2])
        else:
            return 0.0

class Rules:
    follow_wall_ants = [['near', 'near'], ['near', 'medium'], ['near', 'far'],
                      ['medium', 'near'], ['medium', 'medium'],['medium', 'far'],
                      ['far', 'near'], ['far', 'medium'], ['far', 'far']]
    follow_wall_cons = [['slow', 'left'], ['slow', 'left'], ['slow', 'left'],
                      ['medium', 'right'],['fast', 'zero'], ['medium', 'right'],
                      ['medium', 'right'], ['medium', 'right'],['fast', 'right']]
    avoid_obstacle_ants = [['near', 'near', 'near'], ['near', 'near', 'medium'], ['near', 'near', 'far'],
               ['near', 'medium', 'near'], ['near', 'medium', 'medium'], ['near', 'medium', 'far'],
               ['near', 'far', 'near'], ['near', 'far', 'medium'], ['near', 'far', 'far'],
               ['medium', 'near', 'near'], ['medium', 'near', 'medium'], ['medium', 'near', 'far'],
               ['medium', 'medium', 'near'], ['medium', 'medium', 'medium'], ['medium', 'medium', 'far'],
               ['medium', 'far', 'near'], ['medium', 'far', 'medium'], ['medium', 'far', 'far'],
               ['far', 'near', 'near'], ['far', 'near', 'medium'], ['far', 'near', 'far'],
               ['far', 'medium', 'near'], ['far', 'medium', 'medium'], ['far', 'medium', 'far'],
               ['far', 'far', 'near'], ['far', 'far', 'medium'], ['far', 'far', 'far']]
    avoid_obstacle_cons = [['slow', 'left'], ['slow', 'right'], ['slow', 'right'],
               ['slow', 'left'], ['medium', 'right'], ['medium', 'right'],
               ['slow', 'zero'], ['medium', 'right'], ['fast', 'right'],
               ['slow', 'left'], ['slow', 'left'], ['slow', 'left'],
               ['medium', 'left'], ['medium', 'left'], ['medium', 'right'],
               ['medium', 'left'], ['medium', 'zero'], ['fast', 'right'],
               ['slow', 'left'], ['slow', 'left'], ['slow', 'left'],
               ['medium', 'left'], ['medium', 'left'], ['medium', 'left'],
               ['medium', 'left'], ['fast', 'left'], ['fast', 'zero']]
    behaviour_ants = [['near', 'near'], ['near', 'medium'], ['near', 'far'],
                      ['medium', 'near'], ['medium', 'medium'],['medium', 'far'],
                      ['far', 'near'], ['far', 'medium'], ['far', 'far']]
    behaviour_cons = [['low', 'high'], ['low', 'high'], ['low', 'high'],
                      ['high', 'medium'], ['low', 'medium'], ['low', 'medium'],
                      ['high', 'low'], ['medium', 'low'], ['medium', 'low']]

    def __init__(self, behaviour=None, action=None):
        if behaviour == 'FW':
            self.ants = Rules.follow_wall_ants
            self.cons = Rules.follow_wall_cons
        elif behaviour == 'AO':
            self.ants = Rules.avoid_obstacle_ants
            self.cons = Rules.avoid_obstacle_cons
        else:
            self.ants = Rules.behaviour_ants
            self.cons = Rules.behaviour_cons
        if action == 'AO':
            Rules.behaviour_cons[-1] = ['low', 'medium']

    def rules_applicable(self, mfv_dict, behaviour=None):
        print(mfv_dict)
        front = []
        back = []
        antecedents = []
        consequents = []
        if behaviour == 'AO':
            front_left = []
            front_center = []
            front_right = []
            for flx, fcx, frx in zip(*mfv_dict):
                if mfv_dict[0][flx] > 0:
                    front_left.append(flx)
                if mfv_dict[1][fcx] > 0:
                    front_center.append(fcx)
                if mfv_dict[2][frx] > 0:
                    front_right.append(frx)
            for fl in front_left:
                for fc in front_center:
                    for fr in front_right:
                        antecedents.append([fl, fc, fr])
                        consequents.append(self.cons[self.ants.index([fl, fc, fr])])
        else:
            for kf, kb in zip(*mfv_dict):
                if mfv_dict[0][kf] > 0:
                    front.append(kf)
                if mfv_dict[1][kb] > 0:
                    back.append(kb)
            for f in front:
                for b in back:
                    antecedents.append([f, b])
                    consequents.append(self.cons[self.ants.index([f, b])])
        return [antecedents, consequents]

class Controller:
    def __init__(self):
        self.distance_sensors_obj = None
        self.speed_obj = None
        #self.rwall_streeing_obj = None
        self.streeing_obj = None
        self.behaviour_obj = None
        self.mfv_dict = None
        self.ants = None
        self.cons = None
        self.behaviour_firing_strength = None
        self.avoid_obstacle_deffuzed = None
        self.follow_wall_deffuzed = None
        self.create_objects()

    def create_objects(self):
        self.distance_sensors_obj = [MembershipFunction([0.0, 0.0, 0.3, 0.8], "near"),
                              MembershipFunction([0.5, 1.0, 1.0, 1.5], "medium"),
                              MembershipFunction([1.0, 1.5, 6.0, 6.0], "far")]
        self.speed_obj = [MembershipFunction([0.0, 0.0, 0.15, 0.25], "slow"),
                           MembershipFunction([0.25, 0.35, 0.45, 0.55], "medium"),
                           MembershipFunction([0.45, 0.55, 0.65, 0.8], "fast")]
        self.streeing_obj = [MembershipFunction([-3.0, -3.0, -2.0, 0], "left"),
                                 MembershipFunction([-2.0, 0.0, 0.0, 2.0], "zero"),
                                 MembershipFunction([0.0, 2.0, 3.0, 3.0], "right")]
        self.behaviour_obj = [MembershipFunction([0.0, 0.15, 0.15, 0.3], "low"),
                               MembershipFunction([0.15, 0.3, 0.3, 0.45], "medium"),
                               MembershipFunction([0.3, 0.45, 0.45, 0.6], "high")]

    def mfv_dict_calc(self, front_left_sensor_reading, front_center_sensor_reading, front_right_sensor_reading,
                      right_front_sensor_reading, right_back_sensor_reading):
        sensor_title = ["near", "medium", "far"]
        obstacle_behaviour_mfv = [obj.membership_function_calc(
            min(front_left_sensor_reading, front_center_sensor_reading, front_right_sensor_reading)) for obj in
            self.distance_sensors_obj]
        right_front_sensor_mfv = [obj.membership_function_calc(right_front_sensor_reading) for obj in
                                  self.distance_sensors_obj]
        right_back_sensor_mfv = [obj.membership_function_calc(right_back_sensor_reading) for obj in
                                 self.distance_sensors_obj]
        front_left_sensor_mfv = [obj.membership_function_calc(front_left_sensor_reading) for obj in
                                 self.distance_sensors_obj]
        front_center_sensor_mfv = [obj.membership_function_calc(front_center_sensor_reading) for obj in
                                   self.distance_sensors_obj]
        front_right_sensor_mfv = [obj.membership_function_calc(front_right_sensor_reading) for obj in
                                  self.distance_sensors_obj]
        obstacle_behaviour_mfv_dict = dict(zip(sensor_title, obstacle_behaviour_mfv))
        front_left_sensor_mfv_dict = dict(zip(sensor_title, front_left_sensor_mfv))
        front_center_sensor_mfv_dict = dict(zip(sensor_title, front_center_sensor_mfv))
        front_right_sensor_mfv_dict = dict(zip(sensor_title, front_right_sensor_mfv))
        right_front_sensor_mfv_dict = dict(zip(sensor_title, right_front_sensor_mfv))
        right_back_sensor_mfv_dict = dict(zip(sensor_title, right_back_sensor_mfv))
        self.mfv_dict = [obstacle_behaviour_mfv_dict, right_front_sensor_mfv_dict, right_back_sensor_mfv_dict,
                         front_left_sensor_mfv_dict, front_center_sensor_mfv_dict,
                         front_right_sensor_mfv_dict]
        print("mfv dict values: ", self.mfv_dict)

    def compute_firing_strength_value(self, mfv_dict):
            result = []
            for ants in self.ants:
                temp = []
                for i, a in enumerate(ants):
                    temp.append(mfv_dict[i][a])
                result.append(min(temp))
            return result

    def defuzz_value(self, firing_strength_value, consx):
        result = []

        objs = [self.speed_obj, self.streeing_obj]

        for c, val in zip(consx, objs):
            result.append(firing_strength_value * [obj for obj in val if obj.labels == c][0].center)
        return result

    def behaviour_defuzz(self):
        if len(self.behaviour_firing_strength) > 2:
            a = max(self.behaviour_firing_strength)
            b = min(self.behaviour_firing_strength)
	    self.behaviour_firing_strength[0] = a
	    self.behaviour_firing_strength[1] = b
        final_speed = (self.behaviour_firing_strength[0] * self.follow_wall_deffuzed[0] + self.behaviour_firing_strength[1]
                       * self.avoid_obstacle_deffuzed[0]) / sum(self.behaviour_firing_strength)
        final_steer = (self.behaviour_firing_strength[0] * self.follow_wall_deffuzed[1] + self.behaviour_firing_strength[1]
                       * self.avoid_obstacle_deffuzed[1]) / sum(self.behaviour_firing_strength)
        return [final_speed, final_steer]

    def calculate_label_value(self, action):
        rules_applicable = Rules(action=action).rules_applicable(self.mfv_dict[0:2])
        self.ants = rules_applicable[0]
        self.cons = rules_applicable[1]
        print("Applicable Rules for Behavior: ", rules_applicable)

        self.behaviour_firing_strength = self.compute_firing_strength_value(self.mfv_dict[0:2])
        if len(self.behaviour_firing_strength) == 1:
            if 'low' in self.cons[0]:
                self.behaviour_firing_strength.insert(self.cons[0].index('low'), 0)
            elif 'medium' in self.cons[0]:
                self.behaviour_firing_strength.insert(self.cons[0].index('medium'), 0)
        print('Behaviour Strength: ', self.behaviour_firing_strength)
        #####################################################################################################
        rules_applicable = Rules('AO', action=action).rules_applicable(self.mfv_dict[3:], 'AO')
        self.ants = rules_applicable[0]
        self.cons = rules_applicable[1]
        print("Applicable Rules for Obstacle Avoidance: ", rules_applicable)

        total_rules_value = self.compute_firing_strength_value(self.mfv_dict[3:])
        print("OA firing strength: ", total_rules_value)
        result = []
        for r_value, c in zip(total_rules_value, self.cons):
            result.append(self.defuzz_value(r_value, c))

        speed = 0.0
        steer = 0.0
        for i in range(len(result)):
            speed += result[i][0]
            steer += result[i][1]
        final_speed = speed / sum(total_rules_value)
        final_steer = steer / sum(total_rules_value)
        self.avoid_obstacle_deffuzed = [final_speed, final_steer]
        ################################################################################################################
        rules_applicable = Rules('FW', action=action).rules_applicable(self.mfv_dict[1:3])
        self.ants = rules_applicable[0]
        self.cons = rules_applicable[1]
        print("Applicable Rules for Follow Wall: ", rules_applicable)

        total_rules_value = self.compute_firing_strength_value(self.mfv_dict[1:3])

        result = []
        for r_value, c in zip(total_rules_value, self.cons):
            result.append(self.defuzz_value(r_value, c))

        speed = 0.0
        steer = 0.0
        for i in range(len(result)):
            speed += result[i][0]
            steer += result[i][1]
        final_speed = speed / sum(total_rules_value)
        final_steer = steer / sum(total_rules_value)
        self.follow_wall_deffuzed = [final_speed, final_steer]
        ################################################################################################################
        return self.behaviour_defuzz()
        
if __name__ == '__main__':
    try:
        # c = Controller()
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        controller(sys.argv[1])
    except rospy.ROSInterruptException:
		pass
