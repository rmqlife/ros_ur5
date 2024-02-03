import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from myBagReader import MyBagReader

def perform_forward_kinematics(joint_angles):
    # Initialize ROS node
    rospy.init_node('forward_kinematics_example', anonymous=True)

    # Initialize MoveIt!
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    # Create a MoveGroupCommander for the manipulator group
    manipulator_group = MoveGroupCommander('manipulator')

    # Set joint values
    manipulator_group.set_joint_value_target(joint_angles)

    # Plan and execute the joint values
    manipulator_group.go()

    # Get the end-effector pose
    pose = manipulator_group.get_current_pose()

    # Extract position (x, y, z)
    position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

    # Extract orientation (quaternion)
    orientation = [pose.pose.orientation.x, pose.pose.orientation.y,
                   pose.pose.orientation.z, pose.pose.orientation.w]

    return position, orientation

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script_name.py /path/to/your/bag_file.bag")
        sys.exit(1)

    bag_reader = MyBagReader(sys.argv[1])

    # Specify the time interval for sampling (0.1 seconds in this example)
    # print(bag_reader.info())
    data = bag_reader.sample_attribute(topic='/scaled_pos_joint_traj_controller/state', time_interval=0.1)
    print(data)

    # Example usage of the forward kinematics function
    joint_angles = [0.1, -0.5, 0.2, -1.2, 1.5, 0.0]
    position, orientation = perform_forward_kinematics(joint_angles)
    print("Position:", position)
    print("Orientation:", orientation)
