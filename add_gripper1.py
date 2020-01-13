#!/usr/bin/env python

#All the imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
import math
from std_msgs.msg import String



def add_gripper(scene, robot, x, y, z, name, scale_x, scale_y, scale_z):
    print ("adding!!",name)
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    box_name = name
    scene.add_box(box_name, box_pose, size=(scale_x, scale_y, scale_z))
    rospy.sleep(2)
    grasping_group = 'endeffector'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.sleep(2)

    print ("added",name,"!")
   





def move_group_python_interface_tutorial():

    print "fileboxit"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    group = moveit_commander.MoveGroupCommander("manipulator")
    
    
    add_gripper(scene,robot,0,0,0,"gripper",1,0.1,0.15)
    

    rospy.spin()





if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
