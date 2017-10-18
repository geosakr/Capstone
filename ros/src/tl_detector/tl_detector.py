#!/usr/bin/python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion #Quaternion for transformation
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tl_ssd_detector.tl_ssd_detector import TLSSDDetector
from tl_cnn_classifier.tl_cnn_classifier import TLCNNClassifier
import tf
import cv2
import yaml
import math
STATE_COUNT_THRESHOLD = 2


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
		
		# initiliaze variables for traffic light state and location.
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # we need 2 traffic lights publisher
        # 1 for the waypoint (location) and for their state RED, GREEN ....
        self.upcoming_traffic_light_wp_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.upcoming_traffic_light_state_pub = rospy.Publisher('/traffic_state', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.tl_detector = TLSSDDetector()
        self.light_classifier = TLCNNClassifier()
        self.listener = tf.TransformListener()



        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.image_index = 0
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        sub7 = rospy.Subscriber('/image_raw', Image, self.image_cb, queue_size=1)

        # Publish bounding box to image_bboxes
		self.image_bboxes_pub = rospy.Publisher("/image_bboxes", Image, queue_size=1)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.DEBUG = False
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.sub2.unregister()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """

        self.image_index += 1
        if self.image_index % 5 != 1:
            return

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        self.upcoming_traffic_light_wp_pub.publish(Int32(light_wp))

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_traffic_light_state_pub.publish(Int32(self.state))
        else:
            self.upcoming_traffic_light_state_pub.publish(Int32(self.last_state))
        self.state_count += 1

    def euclidean_distance(self, x1, y1, x2, y2):

        x = x2 - x1
        y = y2 - y1
        return math.sqrt((x*x) + (y*y))

    def get_closest_waypoint_to_coords(self, pos):
        #return the index of the closest waypoint to pos

        if self.waypoints is not None:
            closest_dist = float('inf')
            closest_idx = 0
            x = pos[0]
            y = pos[1]

            for i, waypoint in enumerate(self.waypoints.waypoints):
                dist = self.euclidean_distance(x, y,
                                               waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)
                if dist < closest_dist:
                    closest_idx = i
                    closest_dist = dist

            return closest_idx


    def get_closest_waypoint(self, pose):

        return self.get_closest_waypoint_to_coords([pose.position.x, pose.position.y])

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get the image center
        Ox = image_width/2.0
        Oy = image_height/2.0

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        if (trans is None) or (rot is None):
            return -1, -1

        # We will use the function cv2.projectPoints to project the 3D point
		# Into a 2D point. This function needs the point to be in a numpy Array
		# and the Transformation matrix as well as the rotation matrix and the camera matrix
		
		# Creating 3D world point with numpy array
        point_in_world_np = np.array([[point_in_world.x, point_in_world.y, point_in_world.z]])

        # Transform quaternion to euler rotation matrix
        rot_mat = tf.transformations.euler_from_quaternion(rot)

        # Rotation vector and Translation vector
        rvec = np.array(rot_mat)
        tvec = np.array(trans)

        # create the camera matrix from the focal lengths and principal point
        camera_mat = np.matrix([[fx,  0, Ox],
                                [ 0, fy, Oy],
                                [ 0,  0,  1]])

        # Distortion coefficients
        distCoeffs = None

        # corresponding point in image from 3D world point
        img_point, jacobian = cv2.projectPoints(point_in_world_np, rvec, tvec, camera_mat, distCoeffs)

        return img_point[0][0][0], img_point[0][0][1]   # Return X and Y

    def get_light_state(self):
        """Determines the current color of the traffic light
        Args:
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if not self.has_image:
            self.prev_light_loc = None
            return False

        tl_state = TrafficLight.UNKNOWN
        image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        t1 = rospy.get_time()
		# the image is in BGR format We change it to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		#use the traffic light detector to get the bounding box
        bbox = self.tl_detector.get_strongest_bbox(image)
        if bbox is not None:
            ymin = int(bbox[0] * image.shape[0])
            xmin = int(bbox[1] * image.shape[1])
            ymax = int(bbox[2] * image.shape[0])
            xmax = int(bbox[3] * image.shape[1])
			# Crop the image and resize it to fit the CNN classifier
            cropped_image = image[ymin:ymax, xmin:xmax, :]
            resized_image = cv2.resize(cropped_image, (24, 72))

            t2 = rospy.get_time()
            self.image_bboxes_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, "rgb8"))
            tl_state = self.light_classifier.get_classification(resized_image)
            if tl_state != TrafficLight.GREEN:
                tl_state = TrafficLight.RED

        return tl_state

    def get_closest_traffic_light(self, pose):

        # Gets the closest traffic light to the current pose.
		# returns the distance, the index and the waypoiny itself. 
        
        traffic_light_positions = self.config['stop_line_positions']
        closest_dist = float('inf')
        index = -1
        for light_position in traffic_light_positions:
            index += 1
            dist = self.euclidean_distance(pose.position.x, pose.position.y,
                                           light_position[0], light_position[1])
            if dist < closest_dist:
                closest_idx = index
                closest_dist = dist
                closest_light_wp = light_position
        return closest_dist, closest_idx, closest_light_wp

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light_wp, state = -1, TrafficLight.UNKNOWN
        car_position = None
        stop_line_wp_index = -1

        if self.pose:
            car_position = self.get_closest_waypoint(self.pose.pose)

        if car_position:
            closest_light_dist, closest_light_idx, light_wp = self.get_closest_traffic_light(self.pose.pose)

            if self.DEBUG:
                rospy.loginfo(team)
                rospy.loginfo("Current vehicle position: [x:{}, y:{}, z:{}]".format(self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z))
                rospy.loginfo("Closest traffic light position: {}".format(light_wp))
                rospy.loginfo("Closest traffic light index: {}".format(self.get_closest_waypoint_to_coords(light_wp)))
                rospy.loginfo("Current vehicle way point: {}".format(car_position))
                rospy.loginfo("Closest traffic light way point distance: {}".format(closest_light_dist))

            if closest_light_dist < 150:
                state = self.get_light_state()
                stop_line_wp_index = self.get_closest_waypoint_to_coords(light_wp)

        return stop_line_wp_index, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
