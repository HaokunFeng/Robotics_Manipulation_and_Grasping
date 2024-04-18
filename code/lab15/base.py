#! /usr/bin/env python

# TODO: import ????????_msgs.msg

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import math
import copy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        # initialize the node
        #rospy.init_node('base_controller', anonymous=True)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        
        #create publisher for velocity commands
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._current_pose = None

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        vel_msg = Twist()
        # TODO: Fill out msg
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        # TODO: Publish msg
        self._cmd_vel_pub.publish(vel_msg)
        #rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        vel_msg = Twist()
        self._cmd_vel_pub.publish(vel_msg)
        #rospy.logerr('Not implemented.')

    def _odom_callback(self, msg):
        self._current_pose = msg.pose.pose

    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self._current_pose is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for /odom message...')
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._current_pose)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        while not rospy.is_shutdown():
            if self._current_pose is None:
                continue
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            current_x = self._current_pose.position.x
            current_y = self._current_pose.position.y
            dist_moved = math.sqrt((current_x - start.position.x)**2 + (current_y - start.position.y)**2)
            
            direction = -1 if distance < 0 else 1
            if dist_moved >= abs(distance):
                break

            self.move(direction * speed, 0)
            rate.sleep()
    
    def _get_yaw(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        rotation_matrix = quaternion_matrix(quaternion)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        return yaw
    
    def remaining_angle(self, current_yaw, target_yaw):
        """
        Computes the remaining angle from current_yaw to target_yaw.
        Args:
            current_yaw: The current yaw angle, in radians, in the range [0, 2*pi].
            target_yaw: The target yaw angle, in radians, in the range [0, 2*pi].
        Returns:
            remaining_angle: The remaining angle to rotate, in radians.
            direction: The direction to rotate (1 for counter-clockwise, -1 for clockwise).
        """
        diff = target_yaw - current_yaw
        diff = diff % (2*math.pi)
        if diff < 0:
            diff += 2*math.pi
        
        direction = 1 if diff < math.pi else -1
        remaining_angle = min(diff, 2*math.pi - diff)

        return remaining_angle, direction

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self._current_pose is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for /odom message...')
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start_yaw = self._get_yaw(self._current_pose.orientation)
        target_yaw = start_yaw + angular_distance
        target_yaw %= 2*math.pi

        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while not rospy.is_shutdown():
            if self._current_pose is None:
                continue
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            current_yaw = self._get_yaw(self._current_pose.orientation)
            remaining_angle, direction = self.remaining_angle(current_yaw, target_yaw)
            
            if remaining_angle <= 0.02:
                break
            
            self.move(0, direction * speed)
            rate.sleep()


