"""
Helper functions for dbw_node
"""

import numpy as np
import rospy

def get_wp_coordinates_matrix(waypoints):
    """
    Given a list of waypoints, returns a numpy matrix with x y coordinates    
    """

    points = []

    # Transform all points
    for waypoint in waypoints:

        x = waypoint.pose.pose.position.x
        y = waypoint.pose.pose.position.y

        points.append([x, y])

    return np.array(points)

def get_cross_track_error(waypoints, current_pose):
    """
    Given waypoints ahead of the car, fits polynomial to them, estimates expected y at current x pose and compares
    that to actual y to compute cross track error - a deviation from expected trajectory    
    """

    origin = waypoints[0].pose.pose.position

    # Get waypoints into a matrix and shift them to origin
    wp_matrix = get_wp_coordinates_matrix(waypoints)
    shifted_wp_matrix = wp_matrix - np.array([origin.x, origin.y])

    # Get angle form waypoints a bit in the future
    offset = 10
    angle = np.arctan2(shifted_wp_matrix[offset, 1], shifted_wp_matrix[offset, 0])

    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotated waypoints to origin, so they are mostly changing in positive x direction rather than y
    rotated_waypoints = np.dot(shifted_wp_matrix, rotation_matrix)

    # Fit a polynomial to waypoints
    degree = 2

    coefficients = np.polyfit(rotated_waypoints[:, 0], rotated_waypoints[:, 1], degree)

    shifted_current_point = np.array([current_pose.position.x - origin.x, current_pose.position.y - origin.y])
    rotated_current_point = np.dot(shifted_current_point, rotation_matrix)

    expected_value = np.polyval(coefficients, rotated_current_point[0])
    actual_value = rotated_current_point[1]

    return -(actual_value - expected_value)
