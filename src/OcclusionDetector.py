#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: edwardahn
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


    def __init__(self, sawyer_dae, show_render):
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
        self.show_render = show_render


    def setup_sensor(self, intrinsics):
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
        self.sensor.SendCommand('setintrinsic 529 525 328 267 0.01 10')
        self.sensor.SendCommand('setdims 640 480')
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
        for index, link in enumerate(robot.GetLinks()):
            print 'Link %d: %s' % (index, link.GetName())


    def set_joint_angle(self, angle, index):
        """
        Set new joint angle for particular joint.

        Args:
            angle: Net joint angle
            index: Joint to be modified.
        """
        if index >= self.robot.GetDOF():
            raise ValueError('Invalid degree of freedom')
        self.robot.SetDOFValues([angle], [index])


    def set_joint_angles(self, angles):
        """
        Set new joint angles for all joints.

        Args:
            angles: List of all new joint angles
        """
        if len(angles) != self.robot.GetDOF():
            raise ValueError('Incorrect number of angles inputted')
        self.robot.SetDOFValues(angles)


    def get_rendered_image(self):
        """
        Return a camera render.
        """
        if self.sensor == None:
            raise RuntimeError('Sensor not set up')
        self.sensor.SimulationStep(0.01)
        data = self.sensor.GetSensorData()
        img = data.imagedata
        if self.show_render:
            plt.imshow(img, origin='lower')
            plt.show()
        return img


def main():
    oc = OcclusionDetector('/home/edwardahn/Documents/modules/occlusion\
/src/sawyer_robot/sawyer_description/urdf/sawyer.dae', True)
    oc.setup_sensor(0)
    img = oc.get_rendered_image()


if __name__ == '__main__':
    main()
