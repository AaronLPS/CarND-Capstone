#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

#from twist_controller import Controller
from yaw_controller import YawController
from speed_controller import SpeedController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = 0# for YawController
        self.steer_ratio = steer_ratio

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.speed_controller = SpeedController(wheel_radius, vehicle_mass, fuel_capacity, accel_limit, decel_limit, brake_deadband)

        # TODO: Subscribe to all the topics you need to
        self.current_linear_vel = None
        self.current_angular_vel = None
        self.proposed_linear_vel = None
        self.proposed_angular_vel = None
        self.dbw_enabled = False
        
        self.twist_cmd_sub = rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_callback, queue_size=1)
        self.current_velocity_sub = rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_callback, queue_size=1)
        self.dbw_enabled_sub = rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_callback, queue_size=1)
#         self.current_pose_sub = rospy.Subscriber("/current_pose", PoseStamped, self.current_pose_callback, queue_size=1)
#         self.base_waypoints_sub = rospy.Subscriber("/base_waypoints", Lane, self.base_waypoints_callback, queue_size=1)
#         self.gpu_ready_sub = rospy.Subscriber("/gpu_ready", Int32, self.gpu_ready_callback, queue_size=1)

        self.loop()
        
    #Call backe functions
#     def get_yaw(self, orientation_q):
#         quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#         roll, pitch, yaw = euler_from_quaternion (quaternion) # roll and pitch are always = 0
#         return yaw
    
#     def base_waypoints_callback(self, msg):
#         waypoints_ = len(msg.waypoints)
#         for i in range(0,20):
#             wp = msg.waypoints[i]
#             x = wp.pose.pose.position.x
#             y = wp.pose.pose.position.y
#             z = wp.pose.pose.position.z
#             yaw = self.get_yaw(wp.pose.pose.orientation)
#             theta_z = wp.twist.twist.angular.z
#             # vehicle-centered coordinates, So only the x-direction is valid
#             vx = wp.twist.twist.linear.x
            
    def twist_cmd_callback(self, msg):
        # in [x, y] ego coord
        self.proposed_linear_vel = msg.twist.linear.x
        self.proposed_angular_vel = msg.twist.angular.z
#         rospy.logwarn("[DBW Node] proposed_angular_vel=%f", self.proposed_angular_vel) 

    def current_velocity_callback(self, msg):
        # in [x, y] ego coord
        self.current_linear_vel = msg.twist.linear.x
        self.current_angular_vel = msg.twist.angular.z
#         rospy.logwarn("[DBW Node] current_linear_vel=%f", self.current_linear_vel) 

        
    def dbw_enabled_callback(self, msg):
        self.dbw_enabled = msg.data
#         rospy.logwarn("dbw_enabled=%d", self.dbw_enabled)


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            if self.proposed_angular_vel is not None and self.current_linear_vel is not None:
                steer = self.yaw_controller.get_steering(self.proposed_linear_vel, 
                                                         self.proposed_angular_vel, 
                                                         self.current_linear_vel)
                throttle, brake = self.speed_controller.get_throttle_brake(self.proposed_linear_vel,
                                                                           self.current_linear_vel,
                                                                           1.0/50.0) #1.0/loop frequency
            else:
                throttle, brake, steer = 0., 0., 0.
#                 rospy.logwarn("[DBW Node] vel = None") 

            self.publish(throttle, brake, steer)
#             rospy.logwarn("[DBW Node] throttle=%f brake=%f steer=%f", throttle, brake, steer) 
            rate.sleep()
            
    # wrap up msg and publish
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
