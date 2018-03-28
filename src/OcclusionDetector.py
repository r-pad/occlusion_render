#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn

This file defines the class OcclusionDetector, which lets users
view a camera-rendered image for the purpose of spotting occlusions
to the end effector.
"""

import matplotlib.pyplot as plt
import numpy as np

from openravepy import *


class OcclusionDetector:
    """
    This class uses OpenRAVE and offscreen_render to render the image
    the camera on top of the Sawyer arm views. From this image it is
    possible to see what parts of the arm occlude the end effector.
    """

    # Link index for head camera of Sawyer
    camera_link_index = 8


    def __init__(self, sawyer_dae):
        """
        Instantiates the OcclusionDetector class and initializes
        the OpenRAVE environment.

        Args:
            sawyer_dae: Path to Sawyer collada file
        """
        self.env = Environment()
        with self.env:
            self.env.Load(sawyer_dae)
            self.robot = self.env.GetRobots()[0]
        self.sensor = None


    def setup_sensor(self, intrinsics): ####### Not using intrinsics atm ######
        """
        Create camera sensor using offscreen_render and set its
        extrinsics and intrinsics.

        Args:
            intrinsics: Camera intrinsics as a list
                (fx, fy, cx, cy, near, far)
        """
        self.sensor = RaveCreateSensor(self.env,
                'offscreen_render_camera')

        # Set intrinsics/resolution and turn sensor on
        self.sensor.SendCommand('setintrinsic 408 408 640 400 0.01 10')
        self.sensor.SendCommand('setdims 1280 800')
        self.sensor.Configure(Sensor.ConfigureCommand.PowerOn)
        self.sensor.SendCommand('addbody sawyer 255 0 0')

        # Set extrinsics
        links = self.robot.GetLinks()
        tf = links[OcclusionDetector.camera_link_index].GetTransform()
        self.sensor.SetTransform(tf)


    def print_robot_info(self):
        """
        Print information about Sawyer robot.
        """
        print 'Robot name: ', self.robot.GetName()
        print 'DOF Values: ', self.robot.GetDOFValues()
        for index, link in enumerate(self.robot.GetLinks()):
            print 'Link %d: %s' % (index, link.GetName())


    def set_joint_angles(self, angles):
        """
        Set new angles for all joint angles and camera head pan. Note
        that OpenRAVE wants these angles in the form of:
            [j0, head_pan, j1, j2, j3, j4, j5, j6]

        Args:
            angles: List of all new joint angles, in the order
                [head_pan, j0, j1, j2, j3, j4, j5, j6]
        """
        if len(angles) != self.robot.GetDOF():
            raise ValueError('Incorrect number of angles inputted')
        head_pan = angles[1]
        angles[1] = angles[0]
        angles[0] = head_pan
        self.robot.SetDOFValues(angles)

        # Set extrinsics (camera head pan angle)
        links = self.robot.GetLinks()
        tf = links[OcclusionDetector.camera_link_index].GetTransform()
        self.sensor.SetTransform(tf)


    def get_rendered_image(self):
        """
        Return a camera render.
        """
        if self.sensor == None:
            raise RuntimeError('Sensor not set up')
        self.sensor.SimulationStep(0.01)
        data = self.sensor.GetSensorData()
        return data.imagedata

