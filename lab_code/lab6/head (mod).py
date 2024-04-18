#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import math
import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LOOK_AT_ACTION_NAME = '/head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi / 2  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi / 2  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi / 4  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi / 2  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        
        # TODO: Wait for both servers
        rospy.loginfo('Waiting for head action servers...')
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
        rospy.loginfo('Head action servers are available.')
        
    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        goal = PointHeadGoal()
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1.0)
        
        # TODO: Send the goal
        rospy.loginfo('Moving head to look at ({}, {}, {}) in frame {}'.format(x, y, z, frame_id))
        self.look_at_client.send_goal(goal)
        
        # TODO: Wait for result
        self.look_at_client.wait_for_result()
        rospy.loginfo('Head movement complete.')
        #rospy.logerr('Not implemented.')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        pan = min(max(pan, self.MIN_PAN), self.MAX_PAN)
        tilt = min(max(tilt, self.MIN_TILT), self.MAX_TILT)
        
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()
        
        # TODO: Set positions of the two joints in the trajectory point
        point.positions = [pan, tilt]
        
        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        
        # TODO: Add joint names to the list
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        # TODO: Add trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # TODO: Send the goal
        rospy.loginfo('Moving head to pan={}, tilt={}'.format(pan, tilt))
        self.pan_tilt_client.send_goal(goal)
        
        # TODO: Wait for result
        self.pan_tilt_client.wait_for_result()
        rospy.loginfo('Head movement complete.')

        #rospy.logerr('Not implemented.')
