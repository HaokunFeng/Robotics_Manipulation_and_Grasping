#! /usr/bin/env python

# TODO: import ????????_msgs.msg
from geometry_msgs.msg import Twist
import rospy


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
        rospy.init_node('base_controller', anonymous=True)
        
        #create publisher for velocity commands
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
        self.velocity_pub.publish(vel_msg)
        #rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        vel_msg = Twist()
        self.velocity_pub.publish(vel_msg)
        #rospy.logerr('Not implemented.')
