#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState # EA
from std_msgs.msg import Float64
from tf import transformations

import math

# robot state variables
position_ = Point()
yaw_ = 0
steer_angle_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 5
desired_position_.y = 8
desired_position_.z = 0
#parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.7

#publishers
pub = None

def clbk_odom(msg):
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

def clbk_joint_state(msg):
    global steer_angle_

    steer_angle_ = msg.position[1]

def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)

def fix_yaw(des_pos):
    global yaw_,pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        #twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        twist_msg.angular.z = err_yaw
        twist_msg.linear.x = 0.1
        print('Steer angle: [%s]' %steer_angle_)
    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)
        
        

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw -yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos >dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        pub.publish(twist_msg)

    else:
        print('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def main():
    global pub

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    sub_joint_state = rospy.Subscriber('/joint_states', JointState, clbk_joint_state)

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

if __name__ == '__main__':
    main()