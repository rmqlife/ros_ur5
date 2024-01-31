import numpy as np
from myRobotMoveit import MyRobotMoveit  # Import MyRobotMoveit module
import rospy
from geometry_msgs.msg import Pose



def stroke_to_pose(strokes, pt_drop, pt_lift, scale=1):
    pose_list=[]    

    for coords in strokes:
        for i, [x, y] in enumerate(coords):
            pt = pt_drop + np.array((x*scale, y*scale, 0))

            if i==0:
                pose_list.append(list(pt + pt_lift))

            pose_list.append(list(pt))

            if  i==len(coords)-1:
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



if __name__=="__main__":
    rospy.init_node('painting', anonymous=True)

    robot = MyRobotMoveit()
    current_pose = robot.get_pose()
    print(current_pose)
    pt = current_pose.position
    pt_drop = np.array([pt.x, pt.y, pt.z-0.1])
    pt_lift = np.array([0, 0, +0.1])

    from strokes.kanji_strokes import load_strokes, normalize_strokes, save_strokes, round_strokes
    strokes = load_strokes('strokes/jia-1.txt')
    strokes = normalize_strokes(strokes, 0.1, 0.1, True)

    # 2d to 3d
    for s in strokes[:3]:
        poses = stroke_to_pose([s], pt_drop, pt_lift) 
        for p in poses:
            print(p)
        print('exe')
        waypoints = pose2waypoint(poses, current_pose.orientation)

        # execute
        
        robot.set_trajectory(waypoints=waypoints)
        
        
    # print(waypoints[3:6])