#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
 
from std_msgs.msg import String


from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
from gazebo_msgs.srv import GetModelState

def Vizualize_Plan(plan1):
  print " Waiting while RVIZ displays plan1..."
  rospy.sleep(0.5)
 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  #print "   Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  #print "   Waiting while plan1 is visualized (again)..."
  rospy.sleep(1)
 
def init_pose_joint():
  print "============ Go Init Pose in Joint"

  print('  Defining position')

  pose_goal = group.get_current_pose().pose
  pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_goal.position.x =0.40
  pose_goal.position.y =-0.10
  pose_goal.position.z =1.35
  group.set_pose_target(pose_goal)

  print('  Computing Path')
  ## Now, we call the planner to compute the plan
  plan1 = group.plan()
  #Vizualize Plan
  Vizualize_Plan(plan1)

  ## Moving to a pose goal
  print('  Executing Path')
  group.go(wait=True)

def go_down():
  print "============ Go Down"
  print('  Defining position')

  downwards_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(downwards_pose)

  downwards_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  
  #initial_pose.position.x =0.40
  #initial_pose.position.y =-0.10
  downwards_pose.position.z = 0.77

  #Create waypoints
  waypoints.append(copy.deepcopy(downwards_pose))
  print('  Computing Path')
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
  Vizualize_Plan(plan1)


  rospy.sleep(1)
  print('  Executing Path')
  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(1)

def bucket_pose_joint():

  print "============ Go Bucket Pose in Joint"

  print('  Defining position')

  bucket_pose = group.get_current_pose().pose
  bucket_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  bucket_pose.position.x =0.53
  bucket_pose.position.y =-0.23
  bucket_pose.position.z =1.2
  group.set_pose_target(bucket_pose)


  print('  Computing Path')
  ## Now, we call the planner to compute the plan
  plan1 = group.plan()
  #Vizualize Plan
  Vizualize_Plan(plan1)

  ## Moving to a pose goal
  print('  Executing Path')
  group.go(wait=True)

def import_bucket_to_RVIZ():
  #x=0.53, y=-0.23,    z=0.78
  p = geometry_msgs.msg.PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.53
  p.pose.position.y = -0.23
  p.pose.position.z = 0.75+0.2/2
  p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))
  #p.pose.orientation.y = -0.014
  #p.pose.orientation.z = 0.05
  scene.add_box('bucket', p, (0.2, 0.2, 0.2))

def update_cube_to_RVIZ(cube_location,name):
  p = geometry_msgs.msg.PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = cube_location[0]
  p.pose.position.y = cube_location[1]
  p.pose.position.z = cube_location[2]
  scene.add_box(name, p, (0.05, 0.05, 0.05))

def import_all_cubes_to_RVIZ():
  for i in range(0,6):  
    cube_location = []
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    blockName = 'cube{}'.format(i)
    resp_coordinates = model_coordinates(blockName,'link')
    
    if resp_coordinates.success == True:
      
      cube_location.append(resp_coordinates.pose.position.x)
      cube_location.append(resp_coordinates.pose.position.y)
      cube_location.append(resp_coordinates.pose.position.z)

      update_cube_to_RVIZ(cube_location,blockName)

def get_cube_location(i):
  try:
      cube_location = []
      model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      blockName = 'cube{}'.format(i)
      resp_coordinates = model_coordinates(blockName,'link')
      
      if resp_coordinates.success == True:        
        cube_location.append(resp_coordinates.pose.position.x)
        cube_location.append(resp_coordinates.pose.position.y)
        cube_location.append(resp_coordinates.pose.position.z)
        print(blockName)
        print("Cube " + str(blockName))
        print("Value of X : " + str(resp_coordinates.pose.position.x))
        print("Value of Y : " + str(resp_coordinates.pose.position.y)) 
        print("Value of Z : " + str(resp_coordinates.pose.position.z))

        update_cube_to_RVIZ(cube_location,blockName)
      else:
        print('No more cubes')
  except rospy.ServiceException as e:
      rospy.loginfo("Get Model State service call failed:  {0}".format(e))
  return cube_location

def cube_pose(cube_location,i):
  print('=========Go to Cube{}'.format(i))
  print('  Defining position')
  print(cube_location)
  cube_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(cube_pose)

  cube_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  cube_pose.position.x =cube_location[0]
  cube_pose.position.y =cube_location[1]
  cube_pose.position.z = cube_location[2]+0.2

  #Create waypoints
  waypoints.append(cube_pose)
  print('  Computing Path')
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(1)
  Vizualize_Plan(plan1)

  print('  Executing Path')
  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(2.)

def go_down():
  print "============ Go Exactly Down"
  print('  Defining position')

  downwards_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(downwards_pose)

  downwards_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  
  downwards_pose.position.x =cube_location[0]
  downwards_pose.position.y =cube_location[1]
  downwards_pose.position.z = 0.77

  #Create waypoints
  waypoints.append(copy.deepcopy(downwards_pose))
  print('  Computing Path')
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0,         # jump_threshold
                                      avoid_collisions = True)
  plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
  Vizualize_Plan(plan1)

  rospy.sleep(1)
  print('  Executing Path')
  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(1)

def end_programm():
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()

def jointStatesCallback(msg):
  currentJointState = JointState()
  global currentJointState
  currentJointState = msg

def close_gripper():
  print('========= Closing Gripper')
  currentJointState = JointState()
  #rospy.init_node('test_publish')
 
  # Setup subscriber
  rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  #print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    #print 'Published!'
    rospy.sleep(0.5)
 
  print 'end!'

def open_gripper():
  print('=========== Opening Gripper')
  currentJointState = JointState()
  
  #rospy.init_node('test_publish')
 
  # Setup subscriber
  rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  #print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    #print 'Published!'
    rospy.sleep(0.5)
 
  print 'end!'

 

#--------------------------------------------
#--------------------------------------------
#               MAIN
#--------------------------------------------
#--------------------------------------------
if __name__=='__main__':

#--------------------------------------------
# Setup
#--------------------------------------------
  ## First initialize moveit_commander and rospy.
  print("============ Starting tutorial setup")
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")

  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.02)
  group.set_goal_tolerance(0.02)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)

#---------------------------------------------------------------
# Moveing Robot Arm
#---------------------------------------------------------------

# Create list where disposed cubes are remembered
  not_disposed_cubes = []
  try:
    
    import_bucket_to_RVIZ()
    import_all_cubes_to_RVIZ()

    init_pose_joint()
    
    # First try to dispose all cubes
    for cube_number in range (0,6):
      cube_location = get_cube_location(cube_number)
      if not cube_location:
        print('\n')
        print('There is no cube{}'.format(cube_number))
        print('\n')
        continue
      init_pose_joint()
      cube_pose(cube_location,cube_number)
      go_down()
      close_gripper()
      bucket_pose_joint()
      open_gripper()
      init_pose_joint()
      # Check if cube was placed in bucket:
      cube_location = get_cube_location(cube_number)
      if cube_location[1] < -0.15:
        print('\n')
        print('!!!Cube sucessfully disposed!!!')
        print('\n')
      else:
        print('\n')
        print('!!!Cube cannot be disposed, go to next cube!!!')
        print('\n')
        not_disposed_cubes.append(cube_number)

    #Try a second time to dispose the leftover cubes
    for cube_number in not_disposed_cubes:
      cube_location = get_cube_location(cube_number)
      if not cube_location:
        print('\n')
        print('There is no cube{}'.format(cube_number))
        print('\n')
        continue
      init_pose_joint()
      cube_pose(cube_location,cube_number)
      go_down()
      close_gripper()
      bucket_pose_joint()
      open_gripper()
      init_pose_joint()
      # Check if cube was placed in bucket:
      cube_location = get_cube_location(cube_number)
      if cube_location[1] < -0.15:
        print('\n')
        print('!!!Cube sucessfully disposed!!!')
        print('\n')
      else:
        print('\n')
        print('!!!Cube cannot be disposed, go to next cube!!!')
        print('\n')
        not_disposed_cubes.append(cube_number)

      print("\n")
      print("===========!!!!===========")
      print('All Cube have been disposed')
      print("===========!!!!===========")
      print("\n")
      init_pose_joint()
      end_programm()
   

  except rospy.ROSInterruptException:
    print('============EXEPTION')
    pass