#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


class NavPath(object):
    def __init__(self):
        rospy.init_node('nav_path_marker')
        self._path = []
        self._last_position = None
        self._marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
            
    def odom_callback(self, msg):
        if msg.pose.pose.position is None:
            return
        position = msg.pose.pose.position
        if self._last_position is None:
            self._last_position = position
            return
        
        distance = self.calculate_distance(position, self._last_position)
        if distance > 0.1:
            self._path.append(position)
            self.publish_marker()
            self._last_position = position
    
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.points = self._path

        self._marker_pub.publish(marker)
    
    def calculate_distance(self, p1, p2):
        return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

def main():
    # ...setup stuff...
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()