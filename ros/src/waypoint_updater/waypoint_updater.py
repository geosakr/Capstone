#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight

import math
from std_msgs.msg import Int32 # Needed for traffic waypoints and traffic state


LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LAG=2 # Number of steps which passes during calculations
def dist(a, b):

    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.stop_waypoint_index = -1
        self.traffic_light_state = TrafficLight.RED
        self.current_velocity = 0.0
        self.max_velocity = rospy.get_param('/waypoint_loader/velocity') * 0.27778
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub=rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/traffic_state', Int32, self.traffic_state_cb, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.current_waypoints_index = 0
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # Get the closest waypoint index.

        if self.base_waypoints is None:
            return
        if self.current_waypoints_index == len(self.base_waypoints.waypoints) - 1:
            return

        curr_pos =  msg.pose.position
        curr_pos.z = 0
        # get the index of closest waypoint to the current position
        prev_dist = dist (
            curr_pos,
            self.base_waypoints.waypoints[self.current_waypoints_index].pose.pose.position
        )
        for i in range(self.current_waypoints_index + 1, len(self.base_waypoints.waypoints)):
            next_dist = dist(curr_pos,self.base_waypoints.waypoints[i].pose.pose.position)
            if next_dist > prev_dist:
                break
            self.current_waypoints_index = i
            prev_dist = next_dist
            
        # How many waypoint are there before we stop.
        waypoints_to_stop = self.stop_waypoint_index - self.current_waypoints_index
        # In case of green light and not enough waypoints then publish LOOKAHEAD waypoint
        if ((self.traffic_light_state == TrafficLight.GREEN and
                waypoints_to_stop < 10
            ) or
            waypoints_to_stop < 0
        ):
            waypoints_to_publish = LOOKAHEAD_WPS
        # else publish the minimum between the regular number and the waypoints to stop.
        else:
            waypoints_to_publish = min(LOOKAHEAD_WPS, waypoints_to_stop)

        if waypoints_to_publish > 1:
            waypoints_to_publish -= 1

        # Generate next LOOKAHEAD_WPS waypoints from current waypoint
        final_waypoints = Lane()
        final_waypoints.header.frame_id = '/world'
        final_waypoints.header.stamp = rospy.Time(0)
        final_waypoints_end_index = min(self.current_waypoints_index + waypoints_to_publish + 1, len(self.base_waypoints.waypoints))
        final_waypoints.waypoints = self.base_waypoints.waypoints[self.current_waypoints_index + 2:final_waypoints_end_index]

        # Call the 2 functions for accelerating and slowing down ro fill up the final_waypoints
        self.accelerate (final_waypoints)
        self.slow_down (final_waypoints)

        self.final_waypoints_pub.publish(final_waypoints)


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        self.base_waypoints_sub.unregister()
        self.set_stop_waypoint_to_last()
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_state = msg.data
        pass
    def traffic_state_cb(self,msg):
        self.traffic_light_state=msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    def velocity_cb(self, msg):
        self.current_velocity=msg.twist.linear.x

    def accelerate(self, waypoints):
        
        # Sets the velocities of waypoints to accelerate up to self.max_velocity

        
        wp_count = len(waypoints.waypoints)
        if wp_count == 0:
            return
            
        # get the difference in velocity
        diff_v = math.fabs(float(self.max_velocity - self.current_velocity))

        if wp_count < 2:
            return

        wp_length = self.distance(waypoints.waypoints, 0, wp_count - 1)
        time_step = diff_v / 1.0
        accel_length = self.current_velocity * time_step + 1.0 * time_step * time_step * 0.5
        accel_count = int(float(wp_count) * accel_length / wp_length)

        if accel_count != 0:
            time_for_step = time_step / accel_count
        else:
            time_for_step = 0
        v = self.current_velocity
        v += LAG * 1.0 * time_for_step
        for i in range(wp_count):
            v += 1.0 * time_for_step
            self.set_waypoint_velocity(waypoints.waypoints, i, min(v, self.max_velocity))

    def slow_down(self, waypoints):
        
        # Set velocities of ending waypoints to make deceleration and stop
        # Same logic as acceleration
 
        wp_count = len(waypoints.waypoints)
        if wp_count == 0:
            return

        path_velocity = self.get_waypoint_velocity(waypoints.waypoints[-1])
        diff_v = path_velocity-0 #0 is the target to stop.

        if wp_count < 2:
            self.set_waypoint_velocity(waypoints.waypoints, 0, 0)
            return

        wp_length = self.distance(waypoints.waypoints, 0, wp_count - 1)
        time_step = diff_v / 1.0
        accel_length = self.current_velocity * time_step + 1.0 * time_step * time_step * 0.5
        accel_count = int(float(wp_count) * accel_length / wp_length)

        if accel_count != 0:
            time_for_step = time_step / accel_count
        else:
            time_for_step = 0
        v = 0.0
        for i in range(wp_count):
            v += 1.0 * time_for_step
            wp_index = wp_count - i - 1
            wp_vel = self.get_waypoint_velocity(waypoints.waypoints[wp_index])
            if wp_vel > v:
                self.set_waypoint_velocity(waypoints.waypoints, wp_index, v)

    def set_stop_waypoint_to_last(self):
        self.stop_waypoint_index = len(self.base_waypoints.waypoints) - 1
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
