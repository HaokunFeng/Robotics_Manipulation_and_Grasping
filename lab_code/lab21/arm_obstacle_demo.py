#!/usr/bin/env python

import rospy
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import robot_api

def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass

def main():
    rospy.init_node('arm_obstacle_demo')
    reference_frame = 'base_link'

    torso_ctl = robot_api.Torso()
    torso_ctl.set_height(0.4)

    planning_scene = PlanningSceneInterface(frame=reference_frame)

    # Create table obstacle
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                        table_x, table_y, table_z)

    # Create divider obstacle
    planning_scene.removeCollisionObject('divider')
    size_x = 0.3 
    size_y = 0.01
    size_z = 0.4 
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = 0 
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.5
    pose1.pose.position.y = -0.3
    pose1.pose.position.z = 0.75
    pose1.pose.orientation.w = 1

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.5
    pose2.pose.position.y = 0.3
    pose2.pose.position.z = 0.75
    pose2.pose.orientation.w = 1

    arm = robot_api.Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 10,
        'num_planning_attempts': 5,
        'replan': False
    }
    error = arm.move_to_pose(pose1, **kwargs)
    if error is not None:
        rospy.logerr('Pose 1 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 1 succeeded')

        # Attach an object to the robot
        frame_attached_to = 'gripper_link'
        frames_okay_to_collide_with = [
            'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
        ]
        planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                                 frame_attached_to, frames_okay_to_collide_with)
        planning_scene.setColor('tray', 1, 0, 1)
        planning_scene.sendColors()

    rospy.sleep(1)
    error = arm.move_to_pose(pose2, **kwargs)
    if error is not None:
        rospy.logerr('Pose 2 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 2 succeeded')

    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')

    # Remove the attached object at the end of the program
    planning_scene.removeAttachedObject('tray')

    rospy.spin()

if __name__ == '__main__':
    main()