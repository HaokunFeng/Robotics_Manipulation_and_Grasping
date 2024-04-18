#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64)
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: get torso joint value
        torso_lift_value = reader.get_joint('torso_lift_joint')

        # TODO: publish torso joint value
        if torso_lift_value is not None:
            torso_pub.publish(torso_lift_value)
            rospy.loginfo("Published torso_lift_joint value: {}".format(torso_lift_value))
        else:
            rospy.logwarn("No value returned for torso_lift_joint")
        rate.sleep()


if __name__ == '__main__':
    main()
    