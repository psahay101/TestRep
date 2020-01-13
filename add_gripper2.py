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


def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = (math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = (math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = (math.atan2(t3, t4))

        return X, Y, Z




def add_gripper(scene, robot, name, scale_x, scale_y, scale_z, eef_link, group):
    print ("adding!!!!!!!!!!!!!!!",name)
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    pose_goal = group.get_current_pose().pose
    euler= quaternion_to_euler(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w)
    pitch=euler[1]
    yaw=euler[2]
    l=0.075
    dx=l*math.cos(yaw)
    dy=l*math.sin(yaw)
    dz=l*math.sin(pitch)
    pose_goal.position.x+=dx
    pose_goal.position.y+=dy
    pose_goal.position.z-=dz
    box_pose.pose.position.x = pose_goal.position.x
    box_pose.pose.position.y = pose_goal.position.y
    box_pose.pose.position.z = pose_goal.position.z
    box_pose.pose.orientation.w=pose_goal.orientation.w
    box_pose.pose.orientation.x=pose_goal.orientation.x
    box_pose.pose.orientation.y=pose_goal.orientation.y
    box_pose.pose.orientation.z=pose_goal.orientation.z

    

    box_name = name
    rospy.sleep(2)
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
    
    eef_link = group.get_end_effector_link()

    
    add_gripper(scene,robot,"gripper", 0.15, 0.075, 0.075, eef_link, group)
    

    rospy.spin()





if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
