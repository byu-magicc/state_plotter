#!/usr/bin/env python
from __future__ import division

import rospy
import tf

import numpy as np

from state_plotter.Plotter import Plotter

from geometry_msgs.msg import PoseStamped

class PlotWrapper:
    """
    PlotWrapper class for connecting ROS topics to the Plotter object
    """
    def __init__(self, update_freq=30, use_degrees=True):

        # Store parameters
        self.use_degrees = use_degrees

        # Setup the plotter window
        self.plotter = Plotter()

        # Define plot names
        plots =  ['x',      'y',        'z',
                 'u',       'v',        'w',
                 'phi',     'theta',    'psi'
                ]

        # Add plots to the window
        for p in plots:
            self.plotter.add_plot(p)

        # Add legends
        self.plotter.add_legend('x')

        # Define input vectors for easier input
        self.plotter.define_input_vector('position',    ['x', 'y', 'z'])
        self.plotter.define_input_vector('velocity',    ['u', 'v', 'w'])
        self.plotter.define_input_vector('orientation', ['phi', 'theta', 'psi'])

        # Subscribe to relevant ROS topics
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_pose)
        rospy.Subscriber('mavros/local_position/velocity', TwistStamped, self.local_velocity)

        # Update the plots
        rate = rospy.Rate(update_freq)
        while not rospy.is_shutdown():
            self.plotter.update_plots()
            rate.sleep()


    def local_pose(self, msg):
        # Extract time
        t = msg.header.stamp.to_sec()

        # Handle position measurements
        position = msg.pose.position
        self.plotter.add_vector_measurement('position', [position.x, position.y, position.z], t)

        # orientation in quaternion form
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Add angles and angular velocities
        self.plotter.add_vector_measurement('orientation', euler, t, rad2deg=self.use_degrees)

    def local_velocity(self, msg):
        # Extract time
        t = msg.header.stamp.to_sec()

        # Handle position measurements
        linear_velocity = msg.pose.position
        self.plotter.add_vector_measurement('velocity', [linear_velocity.x, linear_velocity.y, linear_velocity.z], t)



if __name__ == '__main__':
    rospy.init_node('mavros_plotter', anonymous=False)

    try:
        obj = PlotWrapper()
    except rospy.ROSInterruptException:
        pass
