#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

This file implements a ROS node that takes in camera and joint angle
information to spot occlusions to the end effector by the arm.
Internally, the class OcclusionDetector is used to render output
images.
"""

import argparse

import message_filters
import rospkg
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState

from OcclusionDetector import OcclusionDetector


class OcclusionDetectorNode:
    """
    This class is a wrapper to the OcclusionDetector class that
    handles ROS functionality.
    """

    def __init__(self, show_render):
        """
        Initialize and run node responsible for occlusion detection.
        """
        rospy.init_node('occlusion_detector')

        # Instantiate OcclusionDetector object
        pkg_path = rospkg.RosPack().get_path('occlusion_detection')
        sawyer_dae = '%s/data/sawyer.dae' % pkg_path
        self.detector = OcclusionDetector(sawyer_dae, show_render)

        self.detector.setup_sensor(0)

        # Register callback subscribing to image and camera info
        image_sub = message_filters.Subscriber(
                '/pose_image/image', Image)
        info_sub = message_filters.Subscriber(
                '/pose_image/camera_info', CameraInfo)
        image_sync = message_filters.TimeSynchronizer(
                [image_sub, info_sub], 10)
        image_sync.registerCallback(self.image_callback)

        # Register callback subscribing to joint angles
        rospy.Subscriber('/robot/joint_states', JointState,
                self.joints_callback)


    def spin(self):
        """
        Spin, waiting for callbacks.
        """
        rospy.spin()


    def image_callback(self, image, camera_info):
        self.detector.get_rendered_image()


    def joints_callback(self, joint_state):
        """
        Set new joint angles every time this callback is called.

        Args:
            joint_state: A list of joint angles and gripper angle.
                Note that the gripper angle (last entry to this list)
                is ignored.
        """
        positions = joint_state.position
        joint_angles = [angle for angle in positions[:-1]]
        self.detector.set_joint_angles(joint_angles)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Occlusion Detector Argument Parser')
    parser.add_argument('--disp', dest='disp', type=int, default=1)
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    node = OcclusionDetectorNode(args.disp)
    node.spin()
