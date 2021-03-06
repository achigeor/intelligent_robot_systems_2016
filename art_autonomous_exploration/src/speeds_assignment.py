#!/usr/bin/env python
# from __future__ import division
from __future__ import print_function
import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation


# Class for assigning the robot speeds
class RobotController:
    # Constructor
    def __init__(self):

        # Debugging purposes
        self.print_velocities = rospy.get_param('print_velocities')

        # Where and when should you use this?
        self.stop_robot = False

        # Create the needed objects
        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()
        self.navigation = Navigation()

        self.linear_velocity = 0
        self.angular_velocity = 0

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # The timer produces events for sending the speeds every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
        self.velocity_publisher = rospy.Publisher( \
            rospy.get_param('speeds_pub_topic'), Twist, \
            queue_size=10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

        # Produce speeds
        self.produceSpeeds()

        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity

        # Send the command
        self.velocity_publisher.publish(twist)

        # Print the speeds for debuggind purposes
        if self.print_velocities == True:
            print("[L,R] = [" + str(twist.linear.x) + " , " + \
                  str(twist.angular.z) + "]")

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
        scan = self.laser_aggregation.laser_scan
        linear = 0
        angular = 0
        ############################### NOTE QUESTION ############################
        # Check what laser_scan contains and create linear and angular speeds
        # for obstacle avoidance
        import numpy as np
        # if distance is between 7 and 10m go with max speed
        # frontal radar rays are in the middle of scan matrix
        linear = np.clip(np.min(scan[(len(scan) / 2 - 23):(len(scan) / 2 + 23)]), a_min=0, a_max=7) / 7
        if linear < 0.6:  # Decrease this value to delay change, here is 0.5 meaning that if an obstacle is closer than
            # 3.5m, you need to turn
            # whether to turn left or right depending on lidar values
            if min(scan[0:(len(scan) / 2 - 23)]) < min(scan[(len(scan) / 2 + 23):len(scan)]):
                angular = 1
            else:
                angular = -1
        ##########################################################################
        return [linear, angular]


    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

        # Produce target if not existent
        if self.move_with_target == True and \
                        self.navigation.target_exists == False:
            # Create the commands message
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            # Send the command
            self.velocity_publisher.publish(twist)
            self.navigation.selectTarget()

        # Get the submodule's speeds
        [l_laser, a_laser] = self.produceSpeedsLaser()

        # You must fill these
        self.linear_velocity = 0
        self.angular_velocity = 0

        if self.move_with_target:
            [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
            ############################### NOTE QUESTION ############################
            # You must combine the two sets of speeds. You can use motor schema,
            # subsumption of whatever suits your better.
            if l_laser<0.1: # kati den tou aresei kai pigenei opou nanai kapoies fores
                self.linear_velocity = 0.3*l_laser
                self.angular_velocity = 0.3 * a_laser
            else:
                self.linear_velocity = 0.3*l_goal
                self.angular_velocity = 0.3*a_goal

            ##########################################################################
        else:
            ############################### NOTE QUESTION ############################
            # Implement obstacle avoidance here using the laser speeds.
            # Hint: Subtract them from something constant

            self.linear_velocity = 0.3*l_laser
            self.angular_velocity = 0.3*a_laser
            ##########################################################################

    # Assistive functions
    def stopRobot(self):
        self.stop_robot = True

    def resumeRobot(self):
        self.stop_robot = False
