#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        """
        Run the node until rospy is shutting down, publishing
        the next waypoints ahead of the car.
        """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                num_waypoins = rospy.get_param('~lookahead_wps', LOOKAHEAD_WPS)
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx, N=num_waypoins)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        Obtains the index of the closest waypoint in front of the car
        from the car's current position and the waypoint position Kd-Tree.
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

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

    def publish_waypoints(self, closest_idx, N):
        """
        Publishes the N closest waypoints in front of the car to the final_waypoints topic.
        """
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:(closest_idx + N)]
        self.final_waypoints_pub.publish(lane)

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
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = self.waypoints_to_2d(waypoints.waypoints)
            self.waypoint_tree = KDTree(self.waypoints_2d)
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
