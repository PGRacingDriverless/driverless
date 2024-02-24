import numpy as np
import quaternion
import math
import heapq
from minimal_working_example_no_ros.vehicle_parameters import FSDS_car_params
# pure pursuit algorithm by Thomas Fermi: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html

def get_reference_point(lookahead, polyline, vehicle_position, vehicle_heading, k=5):
    '''
    :param lookahead: radius of lookahead circle
    :param polyline: list of points (x, y) that defines reference path
    :param vehicle_position: (x, y, z) 3d position
    :param vehicle_heading: 3d heading unit vector
    :param k: how many points from desired path we consider (based on distance to vehicle)
    :return: ref_point -> intersection of lookahead circle and reference path
    There might be multiple intersection points (for example on hairpin) so it's necessary to return only the first
    intersection. (check segments in the order of occurence)
    '''
    vehicle_position2d = vehicle_position[:-1]
    vehicle_heading2d = vehicle_heading[:-1]
    # Select only the nearest points from path
    nearest_points = k_nearest_points(base_point=vehicle_position2d, list_of_points=polyline, k=k)
    # calculate origin as rear axle position
    rear_position = calculate_rear_axle_position(vehicle_position, FSDS_car_params['cog_rear_distance'], vehicle_heading)
    # Transform coordinates to rear axle reference frame
    rear_position2d = rear_position[:-1]
    nearest_points = translate_to_new_origin(nearest_points, rear_position2d)
    # Calculate intersections
    intersections = find_intersections(lookahead, nearest_points)
    print("Intersections", intersections)
    if len(intersections) == 0:
        return None
    # Calculate unit vectors pointing from rear axle to intersection points
    directions = calculate_vectors_directions(intersections)
    # calculate dot products between heading and intersection directions
    dot_products = calculate_dot_products(vehicle_heading2d, directions)
    # choose intersection with maximum dot_product as target point
    target_point = intersections[np.argmax(dot_products)]
    # transform target point to original frame
    target_point = translate_to_new_origin([target_point], -rear_position2d)[0]
    return target_point


def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def k_nearest_points(base_point, list_of_points, k):
    '''
    Get k points, nearest to base_point.
    :param base_point:
    :param list_of_points:
    :param k:
    :return: k points from list_of_points, which are nearest to base_point
    '''
    distances = [(index, euclidean_distance(base_point, point)) for index, point in enumerate(list_of_points)]
    distances.sort(key=lambda x: x[1])  # Sort by distance
    result_indices = [index for index, _ in distances[:k]]  # Get indices of k nearest points
    result_indices.sort()  # Sort by original order
    return [list_of_points[index] for index in result_indices]

def max_vector_alignment(base_vector, list_of_vectors):
    # convert to numpy
    base_vector = np.array(base_vector)
    list_of_vectors = np.array(list_of_vectors)
    dot_products = np.dot(list_of_vectors, base_vector)
    index_of_highest_alignment = np.argmax(dot_products)
    vector_with_highest_alignment = list_of_vectors[index_of_highest_alignment]
    return vector_with_highest_alignment

def orientation_to_heading(orientation_quaternion):
    '''
    Transforms orientation quaternion to heading vector.
    :param orientation_quaternion: Quaternionr (Type from Formula Student Driverless Simulator API)
    :return heading: Vehicle heading vector
    '''
    quat_array = np.array([orientation_quaternion.w_val,
                                 orientation_quaternion.x_val,
                                 orientation_quaternion.y_val,
                                 orientation_quaternion.z_val], dtype=np.float32)
    # Normalize quaternion
    normalized_quat = quat_array / np.linalg.norm(quat_array)
    quat = quaternion.quaternion(normalized_quat[0],
                                 normalized_quat[1],
                                 normalized_quat[2],
                                 normalized_quat[3])

    # Rotate the fixed vector [1, 0, 0] using the normalized quaternion
    # [1, 0, 0] vector is initial heading of vehicle
    heading = quaternion.rotate_vectors(quat, np.array([1, 0, 0]))
    return heading

def sign(x):
    if x < 0:
        return -1
    else:
        return 1

def vector_projection(v1, v2):
    # Ensure that v1 and v2 are NumPy arrays
    v1 = np.array(v1)
    v2 = np.array(v2)

    # Calculate the dot product of v1 and v2
    dot_product = np.dot(v1, v2)

    # Calculate the squared norm of v2
    norm_squared = np.linalg.norm(v2) ** 2

    # Calculate the projection using the formula
    projection = (dot_product / norm_squared) * v2

    return projection

def vector_rejection(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)

    projection = vector_projection(v1, v2)
    rejection = v1 - projection
    return rejection


def segment_circle_distance(A, B, C):
    '''
    This function calculates checks if segment connecting points A and B intersects with circle of radius R with origin
    in C.
    :param A: segment start (x, y)
    :param B: segment end (x, y)
    :param C: circle origin (x, y)
    :param R: circle radius (x, y)
    :return float, distance between circle origin and line segment
    '''
    # implementation based on: https://stackoverflow.com/a/1079478
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    AC = C - A
    AB = B - A
    # calculate D (point on line AB closest to C)
    D = vector_projection(AC, AB) + A
    AD = D - A
    k = AD[0] / AB[0] if abs(AB[0]) > abs(AB[1]) else AD[1] / AB[1]
    if k <= 0:
        return euclidean_distance(C, A)
    elif k >= 1:
        return euclidean_distance(C, B)

    return euclidean_distance(C, D)

def is_between(a, b, c):
    '''
    Function returns true if point c lies on segment between a and b
    :param a: tuple of floats (x, y), starting point of a segment
    :param b: tuple of floats (x, y), ending point of a segment
    :param c: tuple of floats (x, y)
    :return: bool, true if c lies on segment between a and b
    '''
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    if math.isclose(euclidean_distance(a, c) + euclidean_distance(c, b), euclidean_distance(a, b)):
        return True
    else:
        return False

def calculate_line_circle_intersections(circle_radius, segment):
    '''
    This function calculates intersections between circle and line segment. Segment end points coordinates [(x1, y1),
     (x2, y2)] should be relative to center of the circle.
    :param circle_radius: radius of circle in meters [m]
    :param segment: end points of segment [(x1, y1), (x2, y2)]
    :return intersections: list of found intersections
    '''
    # implementation of https://mathworld.wolfram.com/Circle-LineIntersection.html
    # get points from segment
    x1, y1 = segment[0]
    x2, y2 = segment[1]
    # define variables
    dx = x2 - x1
    dy = y2 - y1
    dr = np.sqrt(dx**2 + dy**2)
    D = x1*y2 - x2*y1
    r = circle_radius
    # calculate discriminant
    delta = (r**2)*(dr**2) - D**2
    if delta < 0:
        return []
    x1 = (D*dy + sign(dy)*dx*np.sqrt(delta))/dr**2
    x2 = (D*dy - sign(dy)*dx*np.sqrt(delta))/dr**2

    y1 = (-D*dx + np.abs(dy)*np.sqrt(delta))/dr**2
    y2 = (-D*dx - np.abs(dy)*np.sqrt(delta))/dr**2

    if delta == 0:
        return [(x1, y1)]

    intersections = [(x1, y1), (x2, y2)]
    return intersections

def find_intersections(circle_radius, points):
    intersections = []
    for i in range(len(points) - 1):
        point1 = points[i]
        point2 = points[i + 1]
        # Check if there is any intersection
        line_circle_distance = segment_circle_distance(point1, point2, (0, 0))
        if line_circle_distance > circle_radius:
            continue
        pair_intersections = calculate_line_circle_intersections(circle_radius, [point1, point2])
        # check if intersections lie on a segment connecting point 1 and point 2
        pair_intersections = [p for p in pair_intersections if is_between(point1, point2, p)]
        intersections.extend(pair_intersections)
    return intersections

def convert_points_to_line(point1, point2):
    '''
    Function which converts point1 and point2 into line equation coefficients Ax + By + C = 0
    :param point1: tuple of first point coordinates (x1, y1)
    :param point2: tuple of second point coordinates (x2, y2)
    :return: [A, B, C] - line equation coefficients Ax + By + C = 0
    '''
    x1, y1 = point1
    x2, y2 = point2
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    return [A, B, C]

def calculate_rear_axle_position(vehicle_position, rear_offset, heading):
    '''
    This function calculates global position (in Formula Student Driverless Simulator it is relative to the
    initial position).
    :param vehicle_position: Vehicle's center of gravity position (x, y, z) [m]
    :param rear_offset: Rear offset relative to center of gravity [m]
    :param heading: 3d heading vector of vehicle (offset is be moved along direction of this vector) [x, y, z]
    :return rear_axle_position:
    '''
    position = np.array(vehicle_position)
    heading = np.array(heading)
    offset_vector = rear_offset * heading
    rear_axle_position = position + offset_vector
    return rear_axle_position

def translate_to_new_origin(points, origin):
    x, y = origin
    translated_points = [(x1 - x, y1 - y) for x1, y1 in points]
    return translated_points

def calculate_unit_vector(vector):
    vector = np.array(vector)
    return vector/(np.linalg.norm(vector) + 1e-5)

def calculate_vectors_directions(vectors):
    directions = []
    for vector in vectors:
        unit_vector = calculate_unit_vector(vector)
        directions.append(unit_vector)
    return directions

def calculate_dot_products(heading, directions):
    dot_products = []
    for dir_vector in directions:
        dot_p = np.dot(heading, dir_vector)
        dot_products.append(dot_p)
    return dot_products


def rotate_point_to_body_frame(point_global, heading_vector):
    # Normalize heading vector to ensure it's a unit vector
    heading_vector = np.array(heading_vector)
    heading_vector /= np.linalg.norm(heading_vector)

    # Calculate the rotation angle
    theta = np.arctan2(-heading_vector[1], heading_vector[0])

    # Construct the 2D rotation matrix
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])

    # Convert the global point to a numpy array
    point_global = np.array(point_global)

    # Rotate the point to the body frame
    point_body = np.dot(rotation_matrix, point_global)

    return point_body

class PurePursuit:
    def __init__(self, K_dd, wheelbase, min_lookahead=0.5, max_lookahead=4, k=5):
        self.K_dd = K_dd
        self.min_lookahead=min_lookahead
        self.max_lookahead=max_lookahead
        self.k = k # how many reference points we consider (based on distance from vehicle)
        self.L = wheelbase
        self.previous_target = (0, 0)

    def __call__(self, reference_points, position, heading, speed):
        lookahead = np.clip(self.K_dd * speed, self.min_lookahead, self.max_lookahead)
        ref_point = get_reference_point(lookahead, reference_points, position, heading, self.k)
        if ref_point == None:
            ref_point = self.previous_target
        #transform ref_point rear axle frame
        rear_position = calculate_rear_axle_position(position, FSDS_car_params['cog_rear_distance'],
                                                     heading)
        rear_position2d = rear_position[:-1]
        ref_point_rear = translate_to_new_origin([ref_point], rear_position2d)[0]
        ref_point_rear = rotate_point_to_body_frame(ref_point_rear, heading[:-1])
        alpha = np.arctan2(-ref_point_rear[1], ref_point_rear[0])
        steering_angle=np.arctan(2*self.L*np.sin(alpha)/lookahead)
        print("Target_point read ", [round(num, 2) for num in ref_point_rear],
              "Target_point", [round(num, 2) for num in ref_point] , "Global position", [round(num, 2) for num in position] , "Steering angle", steering_angle, "Lookahead", lookahead)
        self.previous_target = ref_point
        return steering_angle


if __name__ == "__main__":
    # check 1
    points = [(0, 0), (10, 10), (20, 20), (50, 50), (100, 100)]
    base_points = (59, 60)
    nearest_points = k_nearest_points(base_points, points, 3)
    print(nearest_points)
    # check 2
    vectors = [(1, 1), (0, 0), (-1, 0), (-1,1), (-1, -1)]
    base_vector = (2, 2)
    max_alignment = max_vector_alignment(base_vector, vectors)
    print("Max alignment", max_alignment)
    # check 3
    v1, v2 = (4, 4), (0, 1)
    v3 = vector_projection(v1, v2)
    print("Vector projection", v3)
    v4 = vector_rejection(v1, v2)
    print("Vector rejection", v4)
    # check 4
    a = (0, 0)
    b = (0, 1)
    c = (1e-5, 0.51)
    print("Is between", is_between(a, b, c))