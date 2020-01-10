# !/usr/bin/env python

# All the imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
from std_msgs.msg import String


def add_objects(scene, robot):
    print ("adding")
    rospy.sleep(10)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 2
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(1, 1, 1))
    rospy.sleep(10)
    print ("added!")


def move_group_python_interface_tutorial():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    group = moveit_commander.MoveGroupCommander("panda_arm")

    add_objects(scene, robot)

    rospy.spin()


if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
