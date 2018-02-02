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
        self.plotter.define_state_vector('MAV', ['x', 'y', 'z', 'u', 'v', 'w', 'phi', 'theta', 'psi'])

        self.plotter.add_plot('x', include_legend=True)
        self.plotter.add_plot('y', include_legend=False)
        self.plotter.add_plot('z', include_legend=False)
        self.plotter.add_plot('u', include_legend=False)
        self.plotter.add_plot('v', include_legend=False)
        self.plotter.add_plot('w', include_legend=False)
        self.plotter.add_plot('phi', include_legend=False)
        self.plotter.add_plot('theta', include_legend=False)
        self.plotter.add_plot('psi', include_legend=False)

        # Subscribe to relevant ROS topics
        c0 = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_pose)

        # Update the plots
        rate = rospy.Rate(update_freq)
        while not rospy.is_shutdown():
            self.plotter.update_plots()
            rate.sleep()


    def local_pose(self, msg):
        # Handle position measurements
        self.plotter.add_measurement('x', msg.pose.position.x, msg.header.stamp.to_sec())
        self.plotter.add_measurement('y', msg.pose.position.y, msg.header.stamp.to_sec())
        self.plotter.add_measurement('z', msg.pose.position.z, msg.header.stamp.to_sec())

        # orientation in quaternion form
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # If requested, convert to degrees
        if self.use_degrees:
            euler = [euler[0]*180/np.pi, euler[1]*180/np.pi, euler[2]*180/np.pi]

        # Add angles and angular velocities
        self.plotter.add_measurement('phi',   euler[0], msg.header.stamp.to_sec())
        self.plotter.add_measurement('theta', euler[1], msg.header.stamp.to_sec())
        self.plotter.add_measurement('psi',   euler[2], msg.header.stamp.to_sec())





if __name__ == '__main__':
    rospy.init_node('mavros_plotter', anonymous=False)

    try:
        obj = PlotWrapper()
    except rospy.ROSInterruptException:
        pass