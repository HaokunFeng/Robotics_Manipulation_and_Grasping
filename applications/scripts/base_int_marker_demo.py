#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import Odometry
import math

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class BaseIntMarker:
    def __init__(self):
        rospy.init_node('base_int_marker_demo')
        self.server = InteractiveMarkerServer("base_int_marker_demo")
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.create_markers()

    def create_markers(self):
        # move forward marker
        forward_marker = self.create_marker("move_forward", 1, 0, 0, "red")
        self.add_marker(forward_marker, self.move_forward_callback)

        # move backward marker
        backward_marker = self.create_marker("move_backward", -1, 0, 0, "green")
        self.add_marker(backward_marker, self.move_backward_callback)

        # turn left marker
        left_marker = self.create_marker("turn_left", 0, 1, 0, "blue")
        self.add_marker(left_marker, self.turn_left_callback)

        # turn right marker
        right_marker = self.create_marker("turn_right", 0, -1, 0, "yellow")
        self.add_marker(right_marker, self.turn_right_callback)
    
    def create_marker(self, name, x, y, z, color):
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = name
        marker.description = name.capitalize()

        # create a cube marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.5
        box_marker.scale.y = 0.5
        box_marker.scale.z = 0.5
        box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = self.get_color(color)

        # create a control for the cube
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box_marker)
        control.interaction_mode = InteractiveMarkerControl.BUTTON

        # add the control to the marker
        marker.controls.append(control)

        # set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        return marker

    def get_color(self, color):
        if color == "red":
            return 1., 0., 0., 1.
        elif color == "green":
            return 0., 1., 0., 1.
        elif color == "blue":
            return 0., 0., 1., 1.
        elif color == "yellow":
            return 1., 1., 0., 1.
        else:
            return 1., 1., 1., 1.
    
    def add_marker(self, marker, callback):
        self.server.insert(marker, callback)
        self.server.applyChanges()
    
    def move_forward_callback(self, feedback):
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)
    
    def move_backward_callback(self, feedback):
        twist = Twist()
        twist.linear.x = -0.2
        self.cmd_vel_pub.publish(twist)

    def turn_left_callback(self, feedback):
        twist = Twist()
        twist.angular.z = 1
        self.cmd_vel_pub.publish(twist)
    
    def turn_right_callback(self, feedback):
        twist = Twist()
        twist.angular.z = -1
        self.cmd_vel_pub.publish(twist)

def main():
    #rospy.init_node('base_int_marker_demo')
    
    base_int_marker = BaseIntMarker()
    wait_for_time()
    rospy.spin()

if __name__ == '__main__':
    main()
    
    

