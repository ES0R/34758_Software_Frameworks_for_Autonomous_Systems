#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
import std_msgs
import numpy as np
import sys
from nav_msgs.srv import GetPlan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import re
import math


def get_qr_point_to_world (msg):
    global pose
    global x_world
    global y_world
    global entered
    global matrix
    global dist

    if msg.pose.position.x != 0 and msg.pose.position.x != 0 and msg.pose.position.x != 0:
        entered = rospy.get_time()
        #get pose relative to the odom topic, same used for navigation
        pose = msg
        pose.header.frame_id = "camera_optical_link"
        pose.header.stamp = rospy.Time()
        pose = listener.transformPose("camera_link",pose)
        pose = listener.transformPose("imu_link",pose)
        pose = listener.transformPose("base_link",pose)
        pose = listener.transformPose("base_footprint",pose)
        dist = math.sqrt(pose.pose.position.x*pose.pose.position.x + pose.pose.position.y*pose.pose.position.y)
        pose = listener.transformPose("odom",pose)
        x_world = pose.pose.position.x
        y_world = pose.pose.position.y

        #print "begin\n"
        #print(x_world,y_world)

        
        #get transform matrix
        translation = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]
        rotation = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        matrix = listener.fromTranslationRotation(translation,rotation)


def get_plan_points_to_qr(x_world, y_world, matrix):
    ret_points = [[0,0,0,0], [0,0,0,0], [0,0,0,0]]
    #points 1 meter away from qr
    vect1 = np.array([0,0,1,1])
    vect2 = np.array([0,0.707,0.707,1])
    vect3 = np.array([0,-0.707,0.707,1])

    vect1 = np.dot(matrix,vect1)
    vect2 = np.dot(matrix,vect2)
    vect3 = np.dot(matrix,vect3)

    vect1_a = [x_world - vect1[0], y_world - vect1[1]]
    vect2_a = [x_world - vect2[0], y_world - vect2[1]]
    vect3_a = [x_world - vect3[0], y_world - vect3[1]]
    #get angle between them 
    vect1_angle = np.rad2deg(math.atan2(vect1_a[1],vect1_a[0]))
    vect2_angle = np.rad2deg(math.atan2(vect2_a[1],vect2_a[0]))
    vect3_angle = np.rad2deg(math.atan2(vect3_a[1],vect3_a[0]))

    ret_points[0] = [vect1[0], vect1[1], vect1_angle,0]
    ret_points[1] = [vect2[0], vect2[1], vect2_angle,0]
    ret_points[2] = [vect3[0], vect3[1], vect3_angle,0]

    return ret_points




def get_qr_data(msg):
    global points
    global saved_pos
    global message
    global still_on_first_two_qr
    global qr_positions
    global getHiddenFrame
    global rightPosition
    global points
    global tryGetToPoints
    global dist
    global stop
    global x_world
    global y_world
    global matrix
    global entered
    global rotate


    if msg.data != "":
        #retrieve data from message
        pattern = r'L\=(\w+)'
        pattern1 = r"\-?\d+\.?\d*"
        m = re.findall(pattern,msg.data)
        data = re.findall(pattern1,msg.data)
        #print(message[0])
        #print(message[0])

        if tryGetToPoints == 1:
            if points[0][3] != int(data[4]):
                rotate = 1
            else:
                rotate = 0


        if message[int(data[4])-1] == "":
            print "entering here\n"
            stop = True
            #ensure_robot_is_stopped()
            '''rate = rospy.Rate(60)
            while rospy.get_time() - entered > 2:
                rate.sleep()'''
            #wait for results
            print "focus\n"
            #rospy.sleep(10)

            print "robot stopped - seeing points\n"
            print "dist"
            print dist

            if tryGetToPoints == 0:
                print "plans\n"
                points = get_plan_points_to_qr(x_world, y_world, matrix)
                points[0][3] = int(data[4])
                points[1][3] = int(data[4])
                points[2][3] = int(data[4])
                tryGetToPoints = 1
            elif tryGetToPoints == 2:            
                '''if dist < 1.5 or dist > 0.6:x
                    print "valid\n"
                    print x_world, y_world
                    print "printed\n"
                    rightPosition[int(data[4])-1] = 1'''

                #rightPosition keeps track if the position got from the qr code is valid or not. To 
                #admit the qr code position is valid the robot will be taken to 0.6-1.5 meters of distance from the target.
                #if rightPosition[int(data[4])-1] == 1:
                if still_on_first_two_qr < 2:
                    saved_pos[still_on_first_two_qr][0] = x_world
                    saved_pos[still_on_first_two_qr][1] = y_world
                    saved_pos[still_on_first_two_qr][2] = float(data[0])
                    saved_pos[still_on_first_two_qr][3] = float(data[1])
                    saved_pos[still_on_first_two_qr][4] = int(data[4])
                    still_on_first_two_qr+=1
                #save qr message to the message array that saves all messages from all qr codes 
                message[int(data[4])-1] = m[0]

                #save qr positions
                for l in range(2):
                    spot = -1
                    for i in range(5):
                        #last element having: a '0' means nothing written in the line, a '-1' means qr position 
                        #written but haven't been there yet, a '1' means qr position written and have been there
                        if qr_positions[i][0] == float(data[l*2]) and qr_positions[i][1] == float(data[1+(l*2)]):
                            if l == 0:
                                qr_positions[i][2] = 1
                            break
                        if qr_positions[i][2] == 0:
                            spot = i

                        if i == 4:
                            if spot != -1:
                                qr_positions[spot][0] = float(data[l*2])
                                qr_positions[spot][1] = float(data[1+(l*2)])
                                qr_positions[spot][2] = -1

                if still_on_first_two_qr == 2:
                    still_on_first_two_qr+=1

                    #get hidden frame
                    getHiddenFrame = 1
                elif still_on_first_two_qr < 2:
                    stop = False

                print "sleeping\n"
                #rospy.sleep(100)
                print "stoped sleeping\n"

                '''else:
                    #robot is wandering make it stop
                    stop = True

                    points = get_plan_points_to_qr(x_world, y_world, matrix)
                    #tryGetToPoints = True'''

    else:
        if tryGetToPoints == 1:
            rotate = 1




def get_robot_pose():
    # get pose of the robot
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        break

    return trans, rot



def ensure_robot_is_stopped():
    #basically sends a new goal, which is the current robot position

    # get pose of the robot
    trans, rot = get_robot_pose()

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = trans[0]
    goal_pose.target_pose.pose.position.y = trans[1]
    goal_pose.target_pose.pose.position.z = 0
    #quaternions = tf.transformations.quaternion_from_euler(0 , 0, np.deg2rad(90))
    goal_pose.target_pose.pose.orientation.x = rot[0]
    goal_pose.target_pose.pose.orientation.y = rot[1]
    goal_pose.target_pose.pose.orientation.z = rot[2]
    goal_pose.target_pose.pose.orientation.w = rot[3]

    client.send_goal(goal_pose)
    client.wait_for_result()




def test_plan(goal_x, goal_y):
    # get pose of the robot
    trans, rot = get_robot_pose()

    start = geometry_msgs.msg.PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = trans[0]
    start.pose.position.y = trans[1]

    goal = geometry_msgs.msg.PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time(0)
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y

    rospy.wait_for_service('/move_base/make_plan')
    try:
        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        msg = GetPlan()
        msg.start = start
        msg.goal = goal
        msg.tolerance = 0.05
        res = get_plan(msg.start, msg.goal, msg.tolerance)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return 1

    return 1


def move_to_goal(goal_x, goal_y, angle, wait):
    global rotate

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = goal_x
    goal_pose.target_pose.pose.position.y = goal_y
    goal_pose.target_pose.pose.position.z = 0
    quaternions = tf.transformations.quaternion_from_euler(0 , 0, np.deg2rad(angle))
    goal_pose.target_pose.pose.orientation.x = quaternions[0]
    goal_pose.target_pose.pose.orientation.y = quaternions[1]
    goal_pose.target_pose.pose.orientation.z = quaternions[2]
    goal_pose.target_pose.pose.orientation.w = quaternions[3]
    
    client.send_goal(goal_pose)
    if wait != 0:
        client.wait_for_result()

    if tryGetToPoints == 1:
        #wait for results
        rospy.sleep(0.5)
        if rotate == 1:          
            for i in range(7):
                angle += 45
                quaternions = tf.transformations.quaternion_from_euler(0 , 0, np.deg2rad(angle))
                goal_pose.target_pose.pose.orientation.x = quaternions[0]
                goal_pose.target_pose.pose.orientation.y = quaternions[1]
                goal_pose.target_pose.pose.orientation.z = quaternions[2]
                goal_pose.target_pose.pose.orientation.w = quaternions[3]
                client.send_goal(goal_pose)
                client.wait_for_result()
                rospy.sleep(0.5)
                if rotate == 0:
                    break



def scan_callback(msg):
    global g_range_ahead
    global g_range_ahead1

    tmp=[msg.ranges[0]]
    tmp1=[msg.ranges[0]]

    for i in range(1,16):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-16,len(msg.ranges)):
        tmp1.append(msg.ranges[i])
  
    g_range_ahead = min(tmp)
    g_range_ahead1 = min(tmp1)

    '''for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)'''


def wander(): 
    global stop
    global getHiddenFrame
    global tryGetToPoints
    global g_range_ahead
    global g_range_ahead1
    global points

    stop = False

    g_range_ahead = 1 # anything to start
    g_range_ahead1 = 1 # anything to start
    driving_forward = True
    rate = rospy.Rate(60)
     
    while not rospy.is_shutdown():
        twist = Twist()
        if stop == True:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)

        if getHiddenFrame == 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            break

        if tryGetToPoints == 1:
            for i in range(3):
                print "testing points"
                res = test_plan(points[i][0], points[i][1])
                if res != 1:
                    if len(res.plan.poses) > 0:
                        print "moving to goal"
                        print points[i]
                        print "printed\n"
                        move_to_goal(points[i][0], points[i][1], points[i][2], 1)
                        break
            
            tryGetToPoints = 2
            rospy.sleep(2)
            tryGetToPoints = 0
            stop = False
            continue

        if stop == True:
            continue

        if ((g_range_ahead < 1.5) or (g_range_ahead1 < 1.5)):
          if g_range_ahead > g_range_ahead1:
            twist.angular.z = 1
          else:
            twist.angular.z = -1

          # TURN
          driving_forward = False
          #print "Turn"
         
        else: # we're not driving_forward
          driving_forward = True # we're done spinning, time to go forward!
          #DRIVE
          #print "Drive"

        '''if g_range_ahead < 0.8:
            # TURN
            driving_forward = False
            print "Turn"
       
        else: # we're not driving_forward
            driving_forward = True # we're done spinning, time to go forward!
            #DRIVE
            print "Drive"'''
       
        if driving_forward:
            twist.linear.x = 0.4
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.4
        cmd_vel_pub.publish(twist)
     
        rate.sleep()



def get_hidden_frame():
    global saved_pos

    #get the two vectors
    print "faf"
    print saved_pos
    vect2 = [saved_pos[1][0] - saved_pos[0][0], saved_pos[1][1] - saved_pos[0][1]]
    vect1 = [saved_pos[1][2] - saved_pos[0][2], saved_pos[1][3] - saved_pos[0][3]]
    #get angle between them 
    vect1_angle = math.atan2(vect1[1],vect1[0])
    vect2_angle = math.atan2(vect2[1],vect2[0])
    angle = vect2_angle - vect1_angle
    #get translation
    Tx = saved_pos[0][0] - math.cos(angle)*saved_pos[0][2] + math.sin(angle)*saved_pos[0][3]
    Ty = saved_pos[0][1] - math.sin(angle)*saved_pos[0][2] - math.cos(angle)*saved_pos[0][3]

    return angle,Tx,Ty



def get_position_to_world(goal_x, goal_y, angle, Tx, Ty):
    #use rotation and translation matrixes through the parameters (angle, Tx, Ty) already calculated
    x = math.cos(angle)*goal_x - math.sin(angle)*goal_y + Tx
    y =  math.sin(angle)*goal_x - math.cos(angle)*goal_y + Ty

    return x, y




def get_next_qr_position(angle, Tx, Ty):
    global qr_positions

    #get next qr position from one of the already stored in qr_positions. The ones not visited yet
    #are marked with -1 as already mentioned above
    for i in range(5):
        if qr_positions[i][2] == -1:
            goal_x = qr_positions[i][0]
            goal_y = qr_positions[i][1]
            break

    print"qr_positions"
    print (goal_x, goal_y)

    goal_x, goal_y = get_position_to_world(goal_x,goal_y, angle, Tx, Ty)

    return goal_x, goal_y




def check_qr_found():
    global qr_positions

    num = 0
    for i in range(5):
        if qr_positions[i][2] == 1:
            num+=1

    return num


if __name__ == '__main__':
    try:
        ''' ....... Variables initialization ........ '''
        #array that saves the first two qr points in order to calculate the hidden frame. 
        #saves both real and hidden related points
        saved_pos = [[0,0,0,0,0],[0,0,0,0,0]]

        x_world = 0
        y_world = 0
        message = ["","","","",""]
        still_on_first_two_qr = 0
        qr_positions = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
        getHiddenFrame = 0
        rightPosition = [[0], [0], [0], [0], [0]]
        entered = 0
        matrix = 0
        points = [[0,0,0,0], [0,0,0,0], [0,0,0,0]]
        tryGetToPoints = 0
        dist = 0
        wandering = True
        rotate = 0
        ''' .......................................... '''

         
        pose = geometry_msgs.msg.PoseStamped()

        rospy.init_node( 'final_project_node' )
        rospy.sleep(3)
        listener = tf.TransformListener()

        # subscribe to the position of the qr_code
        rospy.Subscriber( '/visp_auto_tracker/object_position', geometry_msgs.msg.PoseStamped, get_qr_point_to_world)

        # subscribe to the data of the qr_code
        rospy.Subscriber( '/visp_auto_tracker/code_message', std_msgs.msg.String, get_qr_data)

        #subscribe to the laser scan
        scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

        #publisher for the robot velocity
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        client.wait_for_server()

        #search for the first 2 qr codes
        wander()
        wandering = False

        print "---getting hidden frame----"
        angle,Tx,Ty = get_hidden_frame()

        qr_found = 2
        while qr_found < 5:
            goal_x, goal_y = get_next_qr_position(angle, Tx, Ty)
            ensure_robot_is_stopped()
            print(goal_x, goal_y)

            #executes plan
            move_to_goal(goal_x, goal_y, 90, 1)

            rospy.spin()

            qr_found = check_qr_found()

        print"All Qr_codes found - exiting"
        print"------Message concatenated-----"
        print(message[0] + message[1] + message[2] + message[3] + message[4])

    except rospy.ROSInterruptException:
        pass