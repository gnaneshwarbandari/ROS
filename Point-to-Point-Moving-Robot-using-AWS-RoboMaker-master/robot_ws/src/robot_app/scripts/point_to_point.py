#! /usr/bin/env python

# import ros stuff
import rospy   #python client library for ROS
from sensor_msgs.msg import LaserScan  #package for Laser sensor integrated with robot
from geometry_msgs.msg import Twist, Point  #package defines common geometric primitives such as Points, vectors and poses.
from nav_msgs.msg import Odometry  #package is used ti interact with navigation stack
from tf import transformations   #tf is ued to display coordinate frames of a robot

import math  #package for using mathematical functions

# robot state variables
position_ = Point() 
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 4
desired_position_.y = 4
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# callbacks
def clbk_odom(msg): # to check position and yaw of the robot
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def change_state(state): #to print the state of the robot in console
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def fix_yaw(des_pos): # to fix yaw of the robot
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos): # function to move robot in forward direction
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done(): # final end point of the robot
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main(): # main function
    global pub
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    a=[4,4,0,0]
    b=[0,4,4,0]
    for (i,j) in zip(a,b):
        print(i,j)
        print(position_)
        desired_position_.x=i
        desired_position_.y=j
        change_state(0)
        while not rospy.is_shutdown():
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
                break
                print("done")
                pass
            else:
                rospy.logerr('Unknown state!')
                print("unknown")
                pass
    rate.sleep()       
    print("exit")      

if __name__ == '__main__':
    main()

