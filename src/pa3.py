#!/usr/bin/env python


###########
# Imports #
###########
import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import floor


###################
# PID Controller #
##################

# Parameters inspired by PID node from ROS.org
#   Kp - proportional
#   Ki - integral
#   Kd - derivative

class pid_controller:
    ################
    # Setup Method #
    ################
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_err = 0
        self.derivative_err = 0
        self.last = 0
        self.curr = 0
        self.angle = 0
    # get current control angle
    def get_angle(self):
        return self.angle
    
    # set current control angle:
    ############################################################################################
    # PID calculations roughly based on https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9013 #
    ############################################################################################
    def set_angle(self, curr):
        self.last = self.curr
        self.curr = curr

        # (proportional term (curr) * dt) + proportional term
        self.integral_err = self.curr * 0.01 + self.curr
        # d/dt
        self.derivative_err = (self.curr - self.last) / 0.02
        # angle - full algorithm
        self.angle = (self.curr * self.Kp) + ((self.curr * 0.02 + self.curr) * self.Ki) + ((self.curr - self.last) / 0.02 * self.Kd)


####################################################
# Wall Follower Class - handles everything but PID #
####################################################
class wall_follower:
    ################
    # Setup Method #
    ################
    def __init__(self):
        #initialize node 
        rospy.init_node('wall_follower', anonymous = True)

        #publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        #subscriber
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        # call movement method and get speed set up, along with twist, etc.
        self.t = Twist()

        # set up PID controller
        self.pid_control = pid_controller(-1, -1.2, 0.00001)

        # set up initial ranges
        self.g_range_ahead = 1
        self.right_range = 1
        self.left_range = 1

        # angle errors
        self.angle_error = 0

        # direction of wall
        self.wall_direction = ''

        # booleans to assist with wall following
        self.keep_wandering = True
        self.wall_nearby = True

        self.state_change_time = rospy.Time.now()

        # driving_forward: forward(true) vs. spin inplace (false)
        #   TRUE: until x seconds pass or get close to an obstacle, go forward
        #   FALSE: until y seconds pass, spin in place
        self.driving_forward = True
        self.rate = rospy.Rate(1)

        self.scaling_rate = rospy.Rate(2)

        # state dictionary (could be expanded)
        self.state_ = 0
        self.state_dict_ = {
            0: 'wandering',
            1: 'following wall',
        }


    ######################
    # LaserScan Comeback #
    ######################
    def scan_cb(self, msg):
        self.g_range_ahead = min(msg.ranges)

        self.expected_distance = 1.0

        self.range_ahead = msg.ranges[floor(len(msg.ranges)/2)]

        self.right_range = min(msg.ranges[0:90])
        self.left_range = min(msg.ranges[270:360])

        if self.left_range > self.right_range:
            self.wall_direction = "left"
        else:
            self.wall_direction = "right"

        if self.range_ahead < 1.1 or self.left_range < 1.1 or self.right_range < 1.1:
            self.wall_nearby = True
        else:
            self.wall_nearby = False

        angle_error = self.expected_distance - self.range_ahead

        self.pid_control.set_angle(angle_error)






    def wandering(self, keep_wandering, imported_stwist):
        # check whether antyhing is closer than x meters or 
        # time for driving foreward is over, then start spinning in place
        if self.driving_forward and keep_wandering:
            #rospy.Time.now() > state_change_time
            if (self.g_range_ahead < 0.95):
                self.state_ = 1
                self.driving_forward = False
                self.keep_wandering = False
                self.state_change_time = rospy.Time.now() + rospy.Duration(1)                
        # check whether time to spin is over, then go back to driving
        elif keep_wandering: # we're not driving_forward
            if rospy.Time.now() > self.state_change_time:
                self.driving_forward = True # we're done spinning, time to go forward!
                self.state_change_time = rospy.Time.now() + rospy.Duration(30)
        
        # Create an all zero Twist() message. Note a new one is created each loop
        twist = Twist()
        print(keep_wandering)

        # Depending on whether we are driving foreward, set linear or angular
        if self.driving_forward:
            twist.linear.x = 0.2
        else:
            twist.angular.z = 1.0
        
        # Publish cmd_vel with the desired motion
        if keep_wandering:
            self.cmd_vel_pub.publish(twist)
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)


        # Sleep for 1/rate seconds
        self.rate.sleep()


    ##################
    # Following Wall #
    ##################
    def following_wall(self):
        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        
        twist = Twist()
        twist.linear.x = 0.05
        if self.range_ahead < 0.15:
            self.state_ = 0

        if self.wall_direction == "left":
            if self.left_range < 0.95:
                print("following wall on left")
                twist.angular.z = 0
                twist.angular.z = self.pid_control.get_angle()
        elif self.wall_direction == "right":
            if self.right_range < 0.95:
                print("following wall on right")
                twist.angular.z = 0
                twist.angular.z = self.pid_control.get_angle()
        else:
            twist.angular.z = self.pid_control.get_angle()

        if self.range_ahead < self.expected_distance and self.wall_nearby:
            if self.left_range > 0.95:
                print("rotating left")
                self.rotate(2)
            elif self.right_range > 0.95:
                print("rotating right")
                self.rotate(1)

        if self.right_range > 0.95 and self.left_range > 0.95 and self.range_ahead > 0.95:
            self.keep_wandering = True
            self.state_ = 0
   

        self.cmd_vel_pub.publish(twist)


    ############
    # Rotating #
    ############

    # 2 = rotate to the left, 1 = rotate to the right
    def rotate(self, direction):
        twist = Twist()
        if direction == 2:
            twist.angular.z = -1
            self.cmd_vel_pub.publish(twist)
            self.scaling_rate.sleep()
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
        elif direction == 1:
            twist.angular.z = 1
            self.cmd_vel_pub.publish(twist)
            self.scaling_rate.sleep()
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
    
    ##############################################
    # Prints message with current state of Robot #
    ##############################################
    def current_state(self):
        print("")
        print("**** --- ****")
        if self.state_ == 1:
            print("State: Following")
        elif self.state_ == 0:
            print("State: Wandering")
        print("Current Range Ahead: " + str(self.g_range_ahead))
        print("Left Range: " + str(self.left_range))
        print("Right Range: " + str(self.right_range))
        print("Wall Identified Nearby?: " + str(self.wall_nearby))
        print("Wall Direction: " + self.wall_direction)
        print("PID: " + str(self.pid_control.get_angle()))
        print("**** --- ****")
        print("")
        self.rate.sleep()

    #####################
    # Robot State / run #
    #####################
    def current_state_and_run(self):
        while not rospy.is_shutdown():
            # 0 = wandering
            if self.state_ == 0:
                self.wandering(self.keep_wandering, self.t)
            # 1 = following wall
            if self.state_ == 1:
                self.following_wall()
            self.current_state()
        


if __name__ == '__main__':
    my_little_follower = wall_follower()
    my_little_follower.current_state_and_run()

