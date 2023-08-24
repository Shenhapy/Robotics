#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import radians
from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def all_close(goal, actual, tolerance):
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

   
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm_group"
    group = moveit_commander.MoveGroupCommander(group_name)
    gripper_group = moveit_commander.MoveGroupCommander('hand')

    gripper_group.set_goal_tolerance(0.002)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: " , planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print ("============ End effector: " , eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())

    # for debugging it is useful to print the entire state of the robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")

    # rospy.init_node('__init__', anonymous=True)
    # rospy.Subscriber("chatter", String, callback)
    # desired = float(callback.data)

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = "link_5"
    self.group_names = group_names

    self.go_to_joint_state()

    # xyz = [0.0, desired, 0.0]
    # rpy = self.rad([0, 0, 0])
    # xyzrpy = xyz+rpy
    # self.go_to_pose_goal(xyzrpy)
    xyz = [0.0, 0.0, 0.0]
    rpy = self.rad([0, 0, 0])
    xyzrpy = xyz+rpy
    self.go_to_pose_goal(xyzrpy)

    # Open the gripper and deactivate it
    gripper_group.go([0.03, -0.03], wait=True)  # Deactivate gripper
    gripper_group.stop()

    # Decrease z to reach object
    xyz = [0.0, 0.0, -0.2]
    rpy = self.rad([0, 0, 0])
    xyzrpy = xyz+rpy
    self.go_to_pose_goal(xyzrpy)

    # Activate the gripper and close it
    gripper_group.go([0.018,-0.018], wait=True)  # Activate gripper
    gripper_group.stop()

    # # Go to box position
    # xyz = [0.0, 0.0, -0.2]
    # rpy = self.rad([0, 0, 0])
    # xyzrpy = xyz+rpy
    # self.go_to_pose_goal(xyzrpy)

    # # Open the gripper and deactivate it to throw in the box
    # gripper_group.go([0.03, -0.03], wait=True)  # Deactivate gripper
    # gripper_group.stop()

  def go_to_joint_state(self):
    
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = pi/2
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self,xyzrpy):

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = xyzrpy[3]
    pose_goal.orientation.y = xyzrpy[4]
    pose_goal.orientation.z = xyzrpy[5]
    pose_goal.orientation.w = 0.0
    
    pose_goal.position.x = xyzrpy[0]
    pose_goal.position.y = xyzrpy[1]
    pose_goal.position.z = xyzrpy[2]
    group.set_joint_value_target(pose_goal,True)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  #To convert rpy degrees input to radians
  def rad(self,rpy):
    rpyrad = [radians(rpy[0]),radians(rpy[1]),radians(rpy[2])]
    return rpyrad
 

def main():
  try:
    
    armDriver = MoveGroupPythonIntefaceTutorial()


    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

