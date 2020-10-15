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
rho = 10.0
Ang_Dif = 10.0
# parameters
goal_x = [1.5, 0.0, 0.0, 0.0]
goal_y = [0.0, 0.0, 1.0, 0.0]
goal_theta = [0, 180, 0, -180]
k_rho = 0.3
k_alpha = 1
k_beta = -0.3
Dist_tolerance = 0.1
Ang_tolerance = 1.0

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
    global Ang_Dif
    Ang_Dif = goal_theta[point_index] - theta
    global rho
    rho = math.sqrt(delta_x**2 + delta_y**2)
    alpha = -theta + math.atan2(delta_y, delta_x)*180/math.pi
    beta = goal_theta[point_index] -theta - alpha
    # beta = -theta - alpha
    print("alpha:",alpha)
    print("atan2(delta_y, delta_x)", math.atan2(delta_y, delta_x)*180/math.pi)
    alpha = alpha * math.pi/180
    beta = beta * math.pi/180
    print("Alpha:",alpha)
    vel = Twist()
    vel.linear.x = k_rho * rho
    # vel.angular.z = k_alpha * alpha + k_beta * beta
    vel.angular.z = k_alpha * alpha
    # print("rho: ", rho)
    # print("Angle Dif: ", Ang_Dif)
    # print("Vel", vel.linear.x)
    # print("AngVel: ", vel.angular.z)
    # print("Now is at X: %f, Y: %f, Theta: %f" %(x, y, yaw_angle))
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
        if goal_flag == 1:
            rospy.loginfo("Reached Goal!!!")
        # elif rho <= Dist_tolerance and Ang_Dif <= Ang_tolerance:
        elif rho <= Dist_tolerance:
            point_index += 1
            control_command(point_index)
            print("Reach Goal!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", point_index)
            if point_index == 4:
                goal_flag = 1
        else:
            control_command(point_index)
        rate.sleep()

if __name__ == '__main__':
    start()