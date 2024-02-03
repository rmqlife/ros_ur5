import numpy as np
from myRobotMoveit import MyRobotMoveit
import rospy
from geometry_msgs.msg import Pose
from myConfig import MyConfig
import sys

MAX_STROKE_SIZE = 0.015
MIN_STROKE_SIZE = 0.008
LIFT_HEIGHT = 0.05

def stroke_to_pose(strokes, pt_drop, pt_lift, scale=1, gaussian=True):
    pose_list = []
    for coords in strokes:
        if gaussian:
            from strokes.gaussian_stroke import gaussian_stroke
            p = gaussian_stroke(coords)
        else:
            p = [1 for _ in coords]
        
        print(p)

        for i, [x, y] in enumerate(coords):
            z = MIN_STROKE_SIZE + (MAX_STROKE_SIZE-MIN_STROKE_SIZE)*p[i]
            pt = pt_drop + np.array((x * scale, -y * scale, -z))
            if i == 0:
                pose_list.append(list(pt + pt_lift))

            pose_list.append(list(pt))

            if i == len(coords) - 1:
                pose_list.append(list(pt + pt_lift))
    return pose_list

def pose2waypoint(poses, orientation):
    waypoints = []
    for p in poses:
        new_pose = Pose()
        new_pose.position.x = p[0]
        new_pose.position.y = p[1]
        new_pose.position.z = p[2]
        new_pose.orientation = orientation
        waypoints.append(new_pose)
    return waypoints


def run(filename):
    rospy.init_node('painting', anonymous=True)
    robot = MyRobotMoveit()
    
    joint_configs = MyConfig('../joint_configs.json')
    robot.move_joints(joint_configs.get('paint'))

    current_pose = robot.get_pose()
    # position to write
    pt_drop = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    # position to lift, not to write
    pt_lift = np.array([0, 0, LIFT_HEIGHT])
    from strokes.kanji_strokes import normalize_strokes, load_strokes
    strokes = normalize_strokes(load_strokes(filename), w=0.15, h=0.15, keep_scale=True)

    for s in strokes:
        poses = stroke_to_pose([s], pt_drop, pt_lift, gaussian=True)
        waypoints = pose2waypoint(poses, current_pose.orientation)
        robot.set_trajectory(waypoints)



if __name__ == "__main__":
    run(sys.argv[1])

