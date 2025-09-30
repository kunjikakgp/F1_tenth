#!/usr/bin/env python3
import rclpy
import csv
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
#for visualisation 
from visualization_msgs.msg import Marker
#for transformations 
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_matrix# Returns homogeneous rotation matrix from quaternion
#import qosprofile
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

waypoints = []
        #extracting from csv file 
waypoints = np.genfromtxt(
    "Spielberg_centerline.csv",      # file name
    delimiter=",",        # values separated by commas
    comments="#",         # ignore header/comments starting with #
    dtype=float,
    usecols=(0,1)          # force numeric values
)
waypoints=waypoints[::-1]

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # Initialize transformation matrix as identity until first TF message arrives
        self.T_matrix = np.eye(4)
        self.T = np.eye(4)
        self.xc = 0.0
        self.yc = 0.0
        self.speed=8.0
        self.prevcurve=0.0
        #to see waypoints ahead of the car
        self.counter=0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Best effort for odometry, reliable for static maps
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        #publish to /drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive',10)

        # Subscriber to TF
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile)

        timer_period = 0.08 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.lookahead_marker_pub = self.create_publisher(Marker, '/lookahead_point_marker',5)

    def publish_lookahead_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"   
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pure_pursuit"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.lookahead_marker_pub.publish(marker)
   
        
    def stop_car(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)
        self.get_logger().info("Car stopped")

    def tf_callback(self, msg):
        transformd = None
        for t in msg.transforms:
            if (t.header.frame_id == "map" and t.child_frame_id == "ego_racecar/base_link"):
                transformd = t.transform
                break

        if transformd is None:
            #self.get_logger().warn("map → base_link transform not found")
            return
        # Convert quaternion to rotation matrix
        q = [transformd.rotation.x, transformd.rotation.y,
             transformd.rotation.z, transformd.rotation.w]
        self.T_matrix = quaternion_matrix(q)

        # Set translation
        self.T_matrix[0:3, 3] = [transformd.translation.x,
                             transformd.translation.y,
                             transformd.translation.z]

        self.T= np.linalg.inv(self.T_matrix)
        self.xc = transformd.translation.x
        self.yc = transformd.translation.y

    # def dist(self,l):
    #     d=0.0
    #     i=self.counter
    #     self.distance=[]
    #     while d < l:
    #         x,y,v= waypoints[i]
    #         d=np.hypot(x - self.xc, y - self.yc) 
    #         self.distance.append((x,y,v,d))
    #         i=i+1


    def timer_callback(self):
        print("New callback ")
        pose_msg=AckermannDriveStamped()
        lenc = 0.3302
        delta = 0.5
        #self.dist(L+3)
        xmap=ymap=xcar=ycar=0.0
        if(abs(self.prevcurve)>0.350):
            v=1.0
        elif(abs(self.prevcurve)>0.200):
            v=3.0
        else:
            v=6.0
        L=0.75*v
       
        #get coordinates in map frame 
        for i,(x,y)in enumerate(waypoints[self.counter:],start=self.counter):
            d=np.hypot(x - self.xc, y - self.yc) 
            if(abs(d-L)<=delta) :
                xmap,ymap=x,y
                pose_msg.drive.speed = v
                print("L in range")
                L = d
                self.counter=i
                break
            elif(d<L):
                continue
            else :
                xmap,ymap=x,y
                pose_msg.drive.speed = v
                self.counter=i
                L=d
                print("L greater")
                break
        #convert to car frame 

        var_carframe = self.T @ np.array([xmap, ymap, 0.0, 1.0])
        xcar, ycar = var_carframe[0], var_carframe[1]
        self.publish_lookahead_marker((xmap, ymap))
        curve = (2 * ycar) / (L * L )
        self.prevcurve=curve
        sign=1
        if curve !=0 :
            sign=curve/abs(curve)
        steer = np.arctan(lenc * curve)
        print("steer required  is ",steer)
        # if abs(steer) >max_steer:
        #     steer=sign*max_steer

        # TODO: Publish drive message using AckermannDriveStamped
        
        self.speed=pose_msg.drive.speed
        pose_msg.drive.steering_angle = steer
       
        pose_msg.drive.steering_angle_velocity=0.2
        self.drive_pub.publish(pose_msg)
        print(f"curve: {curve:.3f}, L: {L:.2f}, speed: {pose_msg.drive.speed:.2f}, steer: {steer:.3f}")
       


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass
    finally:
        pure_pursuit_node.stop_car()
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()