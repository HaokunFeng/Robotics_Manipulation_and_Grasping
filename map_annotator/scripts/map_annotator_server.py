#!/usr/bin/env python

import rospy
from map_annotator.msg import UserAction, PoseNames
from std_msgs.msg import String
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class MapAnnotatorServer:
    def __init__(self):
        rospy.init_node('map_annotator_server')
        self.poses = {}

        # Publisher for PoseNames
        self.pose_names_pub = rospy.Publisher('/map_annotator/pose_names', PoseNames, latch=True, queue_size=10)

        # Subscriber for UserAction
        rospy.Subscriber('/map_annotator/user_actions', UserAction, self.user_action_callback)

        # Initialize interactive marker server
        self.marker_server = InteractiveMarkerServer("map_poses")

    def user_action_callback(self, msg):
        if msg.command == UserAction.CREATE:
            self.create_pose(msg.name)
        elif msg.command == UserAction.DELETE:
            self.delete_pose(msg.name)
        elif msg.command == UserAction.GOTO:
            self.goto_pose(msg.name)
        elif msg.command == UserAction.RENAME:
            self.rename_pose(msg.name, msg.updated_name)

    def create_pose(self, pose_name):
        if pose_name not in self.poses:
            self.poses[pose_name] = Pose()  # Initialize pose to (0,0,0)
            self.publish_pose_names()
            self.publish_interactive_marker(pose_name)

    def delete_pose(self, pose_name):
        if pose_name in self.poses:
            del self.poses[pose_name]
            self.publish_pose_names()
            self.marker_server.erase(pose_name)  # Remove interactive marker

    def goto_pose(self, pose_name):
        rospy.loginfo("Sending robot to pose: %s" % pose_name)

    def rename_pose(self, old_name, new_name):
        if old_name in self.poses:
            self.poses[new_name] = self.poses.pop(old_name)
            self.publish_pose_names()
            self.marker_server.erase(old_name)  # Remove old marker
            self.publish_interactive_marker(new_name)

    def publish_pose_names(self):
        pose_names_msg = PoseNames()
        pose_names_msg.poses = list(self.poses.keys())
        self.pose_names_pub.publish(pose_names_msg)

    def publish_interactive_marker(self, pose_name):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = pose_name
        int_marker.pose = self.poses[pose_name]
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        # Create an arrow marker
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        button_control.markers.append(arrow_marker)
        int_marker.controls.append(button_control)

        # Create a text marker to display pose name
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.scale.z = 0.1
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = pose_name
        text_marker.pose.position.z += 0.5  # Display above the arrow

        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)

        self.marker_server.insert(int_marker, self.marker_feedback_callback)
        self.marker_server.applyChanges()

    def marker_feedback_callback(self, feedback):
        pose_name = feedback.marker_name
        if pose_name in self.poses:
            # Update pose position based on feedback
            self.poses[pose_name] = feedback.pose.position
            rospy.loginfo("Updated pose %s position: %s" % (pose_name, str(feedback.pose.position)))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    map_annotator_server = MapAnnotatorServer()
    map_annotator_server.spin()
