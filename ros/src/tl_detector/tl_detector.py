#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import cv2
import yaml

from scipy.spatial import KDTree
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        
        #find closest waypoints
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.closest_wp_index = None
        self.ego_x = None
        self.ego_y = None
        
        self.state_red_count = -1
        
        
        #find stop line
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_waypoints = []
        self.enable_find_stop_lines = False
        self.light_wp = -1
        #
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        
        

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_callback)

 

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.gpu_ready_pub = rospy.Publisher('/gpu_ready', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        
        self.gpu_ready_pub.publish(Int32(1))

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        self.loop()

#         rospy.spin()
        

    
    def loop(self):
        # every 1/3 s, consider the GPU processing capability
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            if self.camera_image is not None:
#                 rospy.loginfo("[tl_detector]:loop->camera_image")
                self.closest_wp_index = self.get_closest_waypoint(self.ego_x, self.ego_y)
                
                closest_light_red_index = -1
                closest_light_red_dist = 1e10
                for i in range(len(self.lights)):
                    light = self.lights[i]
                    dist = self.stop_line_waypoints[i] - self.closest_wp_index
                    if dist >= 0 and dist < 150 and dist  < closest_light_red_dist:
                        closest_light_red_dist = dist
                        closest_light_red_index = i

                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                #Get classification
                state = self.light_classifier.get_classification(cv_image)

                if state == TrafficLight.RED and closest_light_red_index is not -1:
                    rospy.loginfo("Traffic Light: RED")
                    self.light_wp = self.stop_line_waypoints[closest_light_red_index]
                    self.state_red_count = STATE_COUNT_THRESHOLD
                else:
                    rospy.loginfo("Traffic Light: NOT RED")
                    self.state_red_count -= 1

                if self.state_red_count > 0:
                    print("traffic_waypoint=" + str(self.light_wp))
                    self.upcoming_red_light_pub.publish(Int32(self.light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(-1))
#             else:
#                 rospy.loginfo("Wait for Camera Signal")
            rate.sleep()
    
    
    def pose_cb(self, msg):
        self.pose = msg
        self.ego_x = msg.pose.position.x
        self.ego_y = msg.pose.position.y

    def waypoints_cb(self, waypoints):
        if not self.waypoints_2d:
            rospy.loginfo("[tl_detector]: waypoints_cb")
            self.waypoints = waypoints
            # create a KD tree of the base waypoints
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            
            #find stop line
            if not self.enable_find_stop_lines:
                rospy.loginfo("finding_stop_lines")
                for i, line_pos in enumerate(self.stop_line_positions):
                    self.stop_line_waypoints.append(self.get_closest_waypoint(line_pos[0], line_pos[1]))
                    self.enable_find_stop_lines = True
            
    def traffic_cb(self, msg):
        self.lights = msg.lights
        

    def image_callback(self, msg):
        if self.waypoints_2d is not None and self.ego_x is not None:
#             rospy.logwarn("image_callback")
            self.camera_image = msg
            

    def get_closest_waypoint(self, x, y):
        if self.waypoint_tree:
            closest_idx = self.waypoint_tree.query([x, y], 1)[1]
            return closest_idx
        else:
            rospy.loginfo("waypoint_tree =  None " )
            return -1
            
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
