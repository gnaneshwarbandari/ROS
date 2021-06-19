#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan  # package for Laser sensor integrated with robot
from geometry_msgs.msg import Twist    #package defines common geometric primitives such as Points, vectors and poses.
pub = None

def clbk_laser(msg):  #to check in which region the obstacle is present
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    take_action(regions)
    
def take_action(regions): # to take the required direction to move forward
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['front'] > 1 and regions['left'] > 1 and regions['right'] > 1:
        state_description = 'case 1 - nothing'
        linear_x = 0.3
        angular_z = 0
    elif regions['front'] < 1 and regions['left'] > 1 and regions['right'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] > 1 and regions['left'] > 1 and regions['right'] < 1:
        state_description = 'case 3 - right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['left'] < 1 and regions['right'] > 1:
        state_description = 'case 4 - left'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['left'] < 1 and regions['right'] > 1:
        state_description = 'case 6 - front and left'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['left'] > 1 and regions['right'] < 1:
        state_description = 'case 5 - front and right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 1 and regions['left'] < 1 and regions['right'] < 1:
        state_description = 'case 7 - front and left and right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['left'] < 1 and regions['right'] < 1:
        state_description = 'case 8 - left and right'
        linear_x = 0.6
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main(): # main function
    global pub
    
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/myrobot/laser/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
