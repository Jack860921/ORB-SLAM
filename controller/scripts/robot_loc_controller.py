#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# initialize pose
x = 0.0
y = 0.0
yaw_angle = 0.0

# parameters
goal_x = [1, 1, 2, 3]
goal_y = [0, 1, 2, 3]
goal_theta = [-90, 0, 0, 0]
k_rho = 0.3
k_alpha = 0.08
k_beta = -0.03
tolerance = 0.3

def Pose_callback(Odometry):
    global x
    global y
    x = Odometry.pose.pose.position.x
    y = Odometry.pose.pose.position.y
    # print("Now is at: ", x, y)

def imu_callback(orientation):
    global yaw_angle
    yaw_angle = orientation.pose.position.z
    # print(yaw_angle)

def control_command(point_index):
    delta_x = (goal_x[point_index] - x)
    delta_y = (goal_y[point_index] - y)
    theta = yaw_angle
    rho = math.sqrt(delta_x**2 + delta_y**2)
    alpha = -theta + math.atan2(delta_y, delta_x)*180/math.pi
    beta = goal_theta[point_index] -theta - alpha
    # beta = -theta - alpha
    vel = Twist()
    vel.linear.x = k_rho * rho
    vel.angular.z = k_alpha * alpha + k_beta * beta
    # print(delta_x, delta_y)
    print("beta", beta)
    print("alpha: ", alpha)
    global pub
    pub.publish(vel)

def start():
    rospy.Subscriber("/odom_filtered_map", Odometry, Pose_callback)
    rospy.Subscriber("/yaw_filtered_map", PoseStamped, imu_callback)
    
    rospy.init_node('robot_loc_controller')
    rate = rospy.Rate(100) # 10hz
    point_index = 0
    goal_flag = 0
    while not rospy.is_shutdown():
        # if goal_flag == 1:
        #     rospy.loginfo("Reached Goal!!!")
        # elif rho < tolerance and goal_flag == 0:
        #     point_index += 1
        control_command(point_index)
        # else:
        #     control_command(point_index)
        rate.sleep()

if __name__ == '__main__':
    start()