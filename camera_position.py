#!/usr/bin/env python
"""
Script to return Baxter's arms to a "home" position
"""
# rospy - ROS Python API
import rospy
# baxter_interface - Baxter Python API
import baxter_interface
# initialize our ROS node, registering it with the Master
rospy.init_node('Home_Arms')
# create instances of baxter_interface's Limb class
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')
# store the home position of the arms
home_right = {'right_s0': -0.3058654869809496, 'right_s1': -0.9974806724317418, 'right_w0': -0.6668415279369881, 'right_w1': 1.0333828500427433, 'right_w2': 0.4945833019325976, 'right_e0': 1.1714387109542663,
'right_e1': 1.9365792509843693}


home_left = {'left_s0': -0.6175003387478757, 'left_s1': -0.9817292808387332, 'left_w0': 0.49624833041681793,
'left_w1': 1.4016912034966955, 'left_w2': -0.6953573114141918, 'left_e0': -0.9261143314366906, 'left_e1':
 1.260535140872423}
# move both arms to home position
limb_right.move_to_joint_positions(home_right)
limb_left.move_to_joint_positions(home_left)
quit()
