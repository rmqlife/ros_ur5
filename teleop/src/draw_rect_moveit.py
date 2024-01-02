import rospy
from reset_moveit import MyRobotMoveit
from geometry_msgs.msg import Pose, Point

def rectangle_points(current_pose, length=0.1, width=0.05, num_points=20):
    waypoints = []

    # Extract the current position of the end effector
    x = current_pose.position.x
    y = current_pose.position.y
    z = current_pose.position.z
    orientation = current_pose.orientation

    # Compute the corner points of the rectangle
    corners = [
        (x + length / 2, y + width / 2),
        (x + length / 2, y - width / 2),
        (x - length / 2, y - width / 2),
        (x - length / 2, y + width / 2),
    ]

    for i in range(num_points):
        # Interpolate between the corner points to create the rectangle waypoints
        t = float(i) / float(num_points - 1)
        x_interpolated = (1 - t) * corners[0][0] + t * corners[2][0]
        y_interpolated = (1 - t) * corners[0][1] + t * corners[2][1]

        new_pose = Pose()
        new_pose.position.x = x_interpolated
        new_pose.position.y = y_interpolated
        new_pose.position.z = z
        new_pose.orientation = orientation

        waypoints.append(new_pose)

    return waypoints


if __name__ == '__main__':
    try:
        rospy.init_node('draw_rect_moveit', anonymous=True)
        arm = MyRobotMoveit()
        # arm.set_planning_time(10.0)
        # arm.allow_replanning(True)

        current_pose = arm.get_pose()
        waypoints = rectangle_points(current_pose,length=0.3, width=0.1)
        waypoints.append(current_pose)
        arm.set_trajectory(waypoints)

    except rospy.ROSInterruptException:
        pass