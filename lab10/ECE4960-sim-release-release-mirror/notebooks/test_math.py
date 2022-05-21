import numpy as np
import math


def ft2meters(pt):
    meters_in_one_foot = 0.3048
    return (pt[0] * meters_in_one_foot, pt[1] * meters_in_one_foot)

def normalize_angle(a):
    new_a = a
    while (new_a < -180):
        new_a = new_a + 360
    while (new_a >= 180):
        new_a = new_a - 360
    return new_a

def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    delta_rot_1 = normalize_angle(math.degrees(np.arctan2(cur_pose[1]-prev_pose[1],
                                                                        cur_pose[0]-prev_pose[0])) - prev_pose[2])
    delta_trans = math.hypot(cur_pose[1]-prev_pose[1],
                                cur_pose[0]-prev_pose[0])
    delta_rot_2 = normalize_angle(
        cur_pose[2] - prev_pose[2] - delta_rot_1)

    return delta_rot_1, delta_trans, delta_rot_2

def direct_path(start, points):

    # Assume you perfectly go EVERYWHERE, so you can pre-compute all controls
    start = ft2meters(start)
    waypoints = [ft2meters(pt) for pt in points]

    current_loc = (start[0], start[1], 0)
    for i, w in enumerate(waypoints):
        # TODO navigate from current location to waypoint
        print(f"{current_loc} -> {w}")
        sx, sy, sa = current_loc
        dx, dy = w

        da = -90
        # TODO compute the angle for the next waypoint, fix calculations
        if i+1 < len(waypoints):
            (x1,y1)= (np.cos(np.radians(sa)), np.sin(np.radians(sa)))
            (x2,y2) = waypoints[i+1]
            x2 = x2 - dx
            y2 = y2 - dy
            print(f"Vector 1: {x1}, {y1}")
            print(f"Vector 2: {x2}, {y2}")
            denom = np.sqrt(x1**2 + y1**2) * np.sqrt(x2**2 + y2**2)
            numer = x1*x2 + y1*y2
            print(denom)
            print(numer)
            da = np.degrees(np.arccos(numer/denom))

        print(f"Computing control: {(sx,sy,sa)} -> {(dx,dy,da)}")
        (a1,x1,a2) = compute_control((dx,dy,da), (sx,sy,sa))

        print(f"Control: {(a1,x1,a2)}")
        print()


        current_loc = (dx,dy,da)

start_rt = (-4,-3)
points_rt = [
    (-4,0),
    (0,0),
    (1,0),
    (1,-1),
    # (2, -3),
    # (5, -3),
    # (5, -2),
    # (5, 3),
    # (0, 3),
    # (0, 0)
]

direct_path(start_rt, points_rt)