#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from waypoint_updater.msg import WaypointLocation

import math

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
MAX_DECEL = .5  # TODO: fix magic number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.closest_waypoint_pub = rospy.Publisher('closest_waypoint', WaypointLocation, queue_size=1)

        num_waypoins = rospy.get_param('~lookahead_wps', LOOKAHEAD_WPS)
        self.loop(num_waypoins)

    def loop(self, num_waypoins):
        """
        Run the node until rospy is shutting down, publishing
        the next waypoints ahead of the car.
        """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints(N=num_waypoins)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        Obtains the index of the closest waypoint in front of the car
        from the car's current position and the waypoint position Kd-Tree.
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Sanity check.
        if not self.waypoint_tree:
            return -1

        # Query the Kd-Tree for the closest position
        # and obtain the (insertion order) index of the closest point.
        query_result = self.waypoint_tree.query([x, y], k=1)
        closest_idx = query_result[1]

        # To check if closest waypoint is ahead or behind vehicle,
        # we're going to compare the car's position against a hyperplane
        # defined by the vector from the previous coordinate to the 
        # closest one. If the car is in front of the hyperplane
        # perpendicular to the coordinate vector, the closest waypoint
        # is behind the car.
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # In order to determine whether the car's coordinate is in front
        # of the hyperplane, we're going to check whether the vector
        # from the previous to the current (closest) coordinate, as well as
        # the vector of the closest coordinate to the car are pointing
        # in the same direction. If they do, the car's in front.
        # For an intuition, see imgs/hyperplane.jpg in the repo's root.
        car_pos_vect = np.array([x, y])
        closest_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)

        # We're going to use the dot product, which, if positive,
        # implies the same direction of both vectors relative to the hyperplane.
        # (Note that the dot product is zero iff the vectors are perpendicular.)
        val = np.dot(closest_vect - prev_vect, 
                     car_pos_vect - closest_vect)

        # If the closest coordinate is indeed behind the car,
        # we simply pick the next (i.e. following) coordinate in the base list,
        # while treating the list as a circular buffer.
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, N):
        """
        Publishes the N closest waypoints in front of the car (with respect to
        the velocity required to handle traffic lights) to the final_waypoints topic.
        """
        final_lane, closest_idx = self.generate_lane(N)
        self.final_waypoints_pub.publish(final_lane)

        if self.pose and self.base_lane:
            wp = WaypointLocation()
            wp.header = self.pose.header
            wp.index = closest_idx
            wp.pose = self.base_lane.waypoints[closest_idx].pose.pose
            self.closest_waypoint_pub.publish(wp)

    def generate_lane(self, N):
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + N
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        lane = Lane()
        # TODO: What about the header? (was: lane.header = self.base_lane.header)
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane, closest_idx

    def decelerate_waypoints(self, waypoints, closest_idx):
        # Having immutable data is nice to begin with, but more importantly:
        # Since /base_waypoints publishes only once, we should not modifying that list
        # directly. Instead, we create a copy of adjusted waypoints.
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            # Two waypoints back from the line so front of car stops at the line.
            # The "-2" offset
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)

            # TODO: The square root is a pretty nasty approach (stops abruptly) - use a sinusoidal approach instead?
            velocity = math.sqrt(2 * MAX_DECEL * dist)
            if velocity < 1.:  # TODO: Fix magic number (lower velocity threshold)
                velocity = 0.

            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        """
        Stores the car's current pose from the /current_pose topic.
        """
        self.pose = msg
        pass

    @staticmethod
    def waypoints_to_2d(waypoints):
        """
        Converts waypoints of the Lane to 2D coordinates.
        """
        return [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                for waypoint in waypoints]

    def waypoints_cb(self, waypoints):
        """
        Receives the map waypoints from the /base_waypoints topic
        and converts them to a Kd-Tree for parsing.
        """
        # The /base_waypoints message is expected to be published exactly once.
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = self.waypoints_to_2d(waypoints.waypoints)
            self.waypoint_tree = KDTree(self.waypoints_2d)
        pass

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        def dl(a, b): 
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
