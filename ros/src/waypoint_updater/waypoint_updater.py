#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math

from std_msgs.msg import Int32
from scipy.spatial import KDTree # for finding closest waypoints
import numpy as np
from tf.transformations import euler_from_quaternion


LOOKAHEAD_WAYPOINTS = 200 # Number of waypoints taht will be published.

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb, queue_size=1)

        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Message info
        self.frame_id = None
        self.msg_seq = 1
        # Car status: Position, orientation
        self.car_x = None
        self.car_y = None
        self.car_z = None
        self.car_yaw = None

        # waypoints
        self.waypoints = None
        self.waypoints_length = None
        # for find the closest waypoint
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.closest_waypoint_index = None
        self.closest_waypoint_dist = None


        rospy.spin()

    def pose_cb(self, msg):
        # update car status from the message
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        self.car_z = msg.pose.position.z
        self.car_yaw = self.get_yaw(msg.pose.orientation)
        self.frame_id = msg.header.frame_id
        
    def get_yaw(self, orientation_q):
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion (quaternion) # roll and pitch are always = 0
        return yaw

    def current_velocity_cb(self, msg):
        # update the velocity
        self.current_velocity = math.sqrt((msg.twist.linear.x)**2 + (msg.twist.linear.y)**2)

    def waypoints_cb(self, msg):
        # update waypoint
        self.waypoints = msg.waypoints
        self.waypoints_length = len(self.waypoints)
        #self.waypoints_backup = copy.deepcopy(self.waypoints)
        if not self.waypoints_2d:
            rospy.loginfo('way_call 2')
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_idx(self):
        closest_idx = self.waypoint_tree.query([self.car_x, self.car_y], 1)[1]
        # Check the closest is ahead or behind
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx -1]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect  = np.array([self.car_x, self.car_y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Loop function, calculate the final waypoint
    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            if self.waypoints is not None and self.ego_x is not None:
                self.closest_waypoint_index = self.get_closest_waypoint_idx();
                planned_vel = self.get_waypoint_velocity(self.waypoints[self.closest_waypoint_index])
                current_vel = self.current_velocity

                #prepare waypoints ahead
                final_waypoints = []
                for i in range(LOOKAHEAD_WAYPOINTS):
                    final_waypoints.append(self.waypoints[ (self.closest_waypoint_index + i) % self.waypoints_length ])
                #prepare the msg
                lane_msg = Lane()
                lane_msg.header.seq = self.msg_seq
                lane_msg.header.frame_id = self.frame_id
                lane_msg.header.stamp = rospy.Time.now()
                lane_msg.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane_msg)
                self.msg_seq += 1
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
