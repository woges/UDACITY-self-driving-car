#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 1

def distance_between_points(x1, y1, x2, y2):
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def ClosestWaypoint(x, y, waypoints):

    closestLen = 10000000; #large number
    closestWaypoint_index = 0

    for i in range(len(waypoints)):
        map_x = waypoints[i].pose.pose.position.x;
        map_y = waypoints[i].pose.pose.position.y;
        dist = distance_between_points(x,y,map_x,map_y)

        if(dist < closestLen):
            closestLen = dist;
            closestWaypoint_index = i;

    return closestWaypoint_index;

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.light_classifier = TLClassifier()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.stop_line_waypoints = []
        self.stop_line_positions = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.listener = tf.TransformListener()
        print("Listening")

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if self.waypoints != None:
            # List of positions that correspond to the line to stop in front of for a given intersection
            self.stop_line_positions = self.config['stop_line_positions']
            for stop_line in self.stop_line_positions:
                self.stop_line_waypoints.append(ClosestWaypoint(stop_line[0],stop_line[1],self.waypoints.waypoints))
            self.sub2.unregister()

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        #print("state2: ", state)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 1
            self.state = state
            #rospy.loginfo('self.state_count %s,    upcoming_red_light_pub if: %s',self.state_count, format(self.last_wp))
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            #light_wp = light_wp if state == TrafficLight.RED else -1
            light_wp = -1 if state == TrafficLight.GREEN or state == TrafficLight.UNKNOWN else light_wp
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #rospy.loginfo('self.state_count %s,    upcoming_red_light_pub elif: %s',self.state_count, format(self.last_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            #rospy.loginfo('self.state_count %s,    upcoming_red_light_pub else: %s',self.state_count, format(self.last_wp))

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        #return ClosestWaypoint(pose.position.x, pose.position.y, self.waypoints.waypoints)
        pass

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.waypoints == None:
            return -1, TrafficLight.UNKNOWN
        light = None

        if(self.pose):
            car_position = ClosestWaypoint(self.pose.pose.position.x, self.pose.pose.position.y, self.waypoints.waypoints)

            closestLen = 10000000; #large number
            for wp in self.stop_line_waypoints:
                dist = wp - car_position
                if(dist < closestLen and dist > 0):
                    closestLen = dist;
                    light_wp = wp;
                    light = self.lights[self.stop_line_waypoints.index(wp)]
                    #rospy.loginfo('light_wp %s',light_wp)
                    #rospy.loginfo('car_position %s',car_position)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            #state = light.state #Comment once classifier works
            state = self.get_light_state(light) #Uncomment once classifier works
            #print("traffic light state: ", state)
            return light_wp, state
        

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
