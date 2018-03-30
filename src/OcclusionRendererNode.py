#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

This file implements a ROS node that takes in camera and joint angle
information to spot occlusions to the end effector by the arm.
Internally, the class OcclusionRenderer is used to render output
images.
"""

import message_filters
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, JointState

from OcclusionRenderer import OcclusionRenderer


class OcclusionRendererNode:
    """
    This class is a wrapper to the OcclusionRenderer class that
    handles ROS functionality.
    """

    def __init__(self):
        """
        Initialize and run node responsible for occlusion rendering.
        """
        self.bridge = CvBridge()
        rospy.init_node('occlusion_renderer')

        # Instantiate OcclusionRenderer object
        self.pkg_path = rospkg.RosPack().get_path('occlusion_render')
        sawyer_dae = '%s/models/sawyer.dae' % self.pkg_path
        self.renderer = OcclusionRenderer(sawyer_dae)
        self.renderer.setup_sensor()

        # Publish renders onto topic
        self.publisher = rospy.Publisher(
                '/pose_image/occlusion_render', Image, queue_size=1)

        # Register callback subscribing to image and camera info
        image_sub = message_filters.Subscriber(
                '/pose_image/image', Image)
        info_sub = message_filters.Subscriber(
                '/pose_image/camera_info', CameraInfo)
        image_sync = message_filters.TimeSynchronizer(
                [image_sub, info_sub], 1)
        image_sync.registerCallback(self.image_callback)

        # Register callback subscribing to joint angles
        rospy.Subscriber('/robot/joint_states', JointState,
                self.joints_callback)


    def image_callback(self, image, camera_info):
        """
        Publish rendering of camera's image view. Note that the
        original image and camera info are not used, but we subscribe
        to them since the rendering must be published at the same
        timestamp as the image.

        Args:
            image: Image that the camera actually sees (ground truth)
            camera_info: Camera information (ex. intrinsics)
        """
        render = self.renderer.get_rendered_image()
        image_message = self.bridge.cv2_to_imgmsg(render, 'rgb8')
        self.publisher.publish(image_message)


    def joints_callback(self, joint_state):
        """
        Set new joint angles every time this callback is called.

        Args:
            joint_state: A list of joint angles and gripper angle.
                Note that the torso angle (last entry to this list)
                is ignored.
        """
        positions = joint_state.position
        joint_angles = [angle for angle in positions[:-1]]
        self.renderer.set_joint_angles(joint_angles)


if __name__ == '__main__':
    node = OcclusionRendererNode()
    rospy.spin()
