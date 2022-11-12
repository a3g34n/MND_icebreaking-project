#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState # EA

rospy.init_node('odometry_publisher')

steer_angle_ = 0
wheel_speed_ = 0

def clbk_joint_state(msg):
    global steer_angle_
    global wheel_speed_
    steer_angle_ = msg.position[1]
    wheel_speed_ = msg.velocity[0]

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
sub_joint_state = rospy.Subscriber('/joint_states', JointState, clbk_joint_state)

odom_broadcaster = tf.TransformBroadcaster()

r_ = 0.1 # wheel radius
d = 0.3 # distance between front wheel center and rear wheels centers
w_d = 0.2 # distance between rear wheels

x = 0.0 # initial x position
y = 0.0 # initial y position

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)
while not rospy.is_shutdown():
    
    v_traction = wheel_speed_ * r_ # linear speed of the traction

    th = math.degrees(steer_angle_) # steering angle in degrees

    vx = cos(th) * v_traction # translational velocity in x
    vy = 0 # translational velocity in y
    vth = sin(th) * (v_traction/d) # vehicle yaw rate

    current_time = rospy.Time.now()
    print('v_traction: [%s]', v_traction)
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()