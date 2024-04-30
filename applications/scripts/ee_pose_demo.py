#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('ee_pose_demo')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            # get the transform from gripper_link to base_link
            trans = buffer.lookup_transform('base_link', 'gripper_link', rospy.Time(0))

            position = trans.transform.translation
            orientation = trans.transform.rotation

            rospy.loginfo("Gripper Pose in base_link frame: Position: {}, Orientation: {}".format(position, orientation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform between gripper_link and bae_link")
        
        rate.sleep()

if __name__ == '__main__':
    main()

