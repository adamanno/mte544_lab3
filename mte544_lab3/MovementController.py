import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from math import *
import numpy as np
import matplotlib.pyplot as plt

from .a_star_skeleton1 import astar ##Check
#from a_star.py import astar

class movementPublisher(Node):

    def __init__(self):
        super().__init__('movementPublisher')
        
        self.get_logger().info('movement_publisher started successfully. Starting.') 
        
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        timer_period = 0.01
        self.create_timer(timer_period, self.move)
        
        
        self.subscription = self.create_subscription(Odometry,'/odom',self.pose_update,10)
        self.subscription  # prevent unused variable warning
        
        #Initialize variables. Set current position to be position 1
        self.curr_pose = Pose()
        self.path = path
        self.count = 0
        self.reached=False
        self.facing1stpoint=False

    #https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434/?fbclid=IwAR0BBayKAjpEEmITIPtteVInc6GgKSd1SAGdO8IyY-cCVw2EckvFH1uXkok
    def euler_from_quaternion(self, x,y,z,w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def pose_update(self, data):
        #Update the current pose value
        #self.curr_pose=data
        self.curr_pose.x = round(data.pose.pose.position.x, 4)
        self.curr_pose.y = round(data.pose.pose.position.y, 4)
        #self.curr_pose.theta = self.quaternionToEuler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.curr_pose.theta = self.euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.curr_pose.theta = round(self.curr_pose.theta, 4)
        #print(data.pose.pose.orientation)
        #print (self.curr_pose)
    
    #Calculates the distance between the current pose and the target pose using Pythagorean theorem.
    def distance_calculator(self, x, y):
        return sqrt(pow((x - self.curr_pose.x), 2) + pow((y - self.curr_pose.y), 2))
 
    #Velocity calculation using proportional gain, kv
    def lin_velocity_calculator(self, x, y):
        #Gain found through experimentation. Too high of a value causes the turtle to overshoot its 
        #point, which means its spins around and miss the target for a while until it can get to it.
        kv = 2
        
        #Linear velocity is just the gain times the distance, this allows for the turtle to move 
        #fast when its far and slow down when its near
        return kv * self.distance_calculator(x, y)
 
    #Calculates the angular distance between the current pose and the target pose using trigonometry.
    def angle_calculator(self, x, y):
        return atan2(y - self.curr_pose.y, x - self.curr_pose.x)

    #Velocity calculation using proportional gain, kw
    def ang_velocity_calculator(self, x, y):
        #Velocity calculation using proportional gain, kw
        #Gain found through experimentation. Too low of a value causes the turtle to not be able to 
        #turn quick enough and thus miss its target
        kw = 0.5
        
        #Angular velocity is just the gain times the required angle, this allows for the turtle to 
        #turn quickly when its facing the wrong direction and slow down when its nearing the right 
        #direction
        return kw * (self.angle_calculator(x, y) - self.curr_pose.theta)

    def move (self):
        vel_msg = Twist()
        ang_tolerance = 0.5
        dist_tolerance = 0.15
        end_dist_tolerance = 0.05
        lin_speed = 0.1
        if (not self.reached):
            if (self.count == 0):
                ang_dist = abs(self.angle_calculator(self.path[self.count][0], self.path[self.count][1]) - self.curr_pose.theta)
                if (ang_dist >= ang_tolerance and not self.facing1stpoint):
                    vel_msg.angular.z = self.ang_velocity_calculator(self.path[self.count][0], self.path[self.count][1])
                    self.velocity_publisher.publish(vel_msg)
                    print("Adjusting heading to face first point")
                else:
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    if (not self.facing1stpoint):
                        self.facing1stpoint=True
                        print("Facing first point")
                    lin_dist = self.distance_calculator(self.path[self.count][0], self.path[self.count][1])
                    if (lin_dist >= dist_tolerance):
                        vel_msg.linear.x = lin_speed
                        vel_msg.angular.z = self.ang_velocity_calculator(self.path[self.count][0], self.path[self.count][1])
                        self.velocity_publisher.publish(vel_msg)
                        print("Travelling to first point")
                    else:
                        vel_msg.linear.x = 0.0
                        self.velocity_publisher.publish(vel_msg)
                        self.count += 1
                        print("Reached First point")
                        print(self.curr_pose)
            elif (self.count != len(self.path)-1):
                lin_dist = self.distance_calculator(self.path[self.count][0], self.path[self.count][1])
                if (lin_dist >= dist_tolerance):
                    vel_msg.linear.x = lin_speed
                    vel_msg.angular.z = self.ang_velocity_calculator(self.path[self.count][0], self.path[self.count][1])
                    self.velocity_publisher.publish(vel_msg)
                    print("Travelling to point", self.count+1)
                else:
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    self.count += 1
                    print (self.count)
                    print("Reached point", self.count)
                    print(self.curr_pose)
            else:
                lin_dist = self.distance_calculator(self.path[self.count][0], self.path[self.count][1])
                if (lin_dist >= end_dist_tolerance):
                    vel_msg.linear.x = self.lin_velocity_calculator(self.path[self.count][0], self.path[self.count][1])
                    if (vel_msg.linear.x > lin_speed):
                        vel_msg.linear.x = lin_speed
                    vel_msg.angular.z = self.ang_velocity_calculator(self.path[self.count][0], self.path[self.count][1])
                    self.velocity_publisher.publish(vel_msg)
                    print("Travelling to end goal")
                else:
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    self.reached=True
                    print("Reached end goal")
                    print(self.curr_pose)


def main(args=None):
    rclpy.init(args=args)  

    global path
    #path = np.asarray( astar(maze, start, end), dtype=np.float)
    #print(path)
    
    #path = [[-1.999938901551247, -0.5000009993037614],[-1.499938901551247, -0.5000009993037614], [-1.137920318878453,-1.8473295208778042]]
    path = [[-1.999938901551247, -0.9000009993037614],[-1.799938901551247, -1.0000009993037614],[-1.599938901551247, -1.3000009993037614],[-1.299938901551247, -1.6000009993037614],[-1.099938901551247, -1.8000009993037614]]
    #path = [[-1.499938901551247, -0.5000009993037614], [-1.099938901551247, -0.5000009993037614],[-0.599938901551247, -0.5000009993037614], [-0.299938901551247, -0.9000009993037614]]
    #path = [[-1.299938901551247, -0.5000009993037614], [-1.099938901551247, -0.5000009993037614],[-0.599938901551247, -0.5000009993037614]]
    #path = [[-1.137920318878453,-1.8473295208778042], [-1.,-2.8473295208778042]]
    Turtlemove = movementPublisher() 
    rclpy.spin(Turtlemove)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Turtlemove.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
