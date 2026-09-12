"""Resolve the initial world model pose without assuming PoseArray order."""
import math


def select_world_pose(poses, start, position_tolerance=.15, yaw_tolerance=.10):
    candidates = []
    for index, pose in enumerate(poses):
        q = pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        distance = math.hypot(pose.position.x - start[0], pose.position.y - start[1])
        error = abs(math.atan2(math.sin(yaw - start[2]), math.cos(yaw - start[2])))
        if distance <= position_tolerance and error <= yaw_tolerance:
            candidates.append((distance + error, index))
    return min(candidates)[1] if candidates else None
