import rospy
from moveit_msgs.msg import RobotState
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from urdf_parser_py.urdf import URDF
import os

if __name__ == "__main__":
    rospy.init_node('attach_custom_end_effector', anonymous=True)

    # Load the UR5 robot's URDF using URDF package
    urdf_path = rospy.get_param('/robot_description')  # Assumes the robot description is set in the ROS parameter server

    # Load the custom end-effector link's URDF
    end_effector_urdf_path = os.path.join(os.path.dirname(__file__), 'cylinder_tool.urdf')
    custom_end_effector_urdf = URDF.from_xml_file(end_effector_urdf_path)

    # Create a PlanningSceneInterface
    scene = PlanningSceneInterface()

    # Define the pose of the custom end-effector link relative to the robot's end-effector
    end_effector_pose = Pose()
    end_effector_pose.position.x = 0.05  # Adjust these values according to your requirements
    end_effector_pose.position.y = 0.0
    end_effector_pose.position.z = 0.0
    end_effector_pose.orientation.w = 1.0

    # Attach the custom end-effector link to the robot's end-effector
    robot_state = RobotState()
    # Define the joint names and positions for your custom end-effector based on its URDF
    robot_state.joint_state.name = ['joint_name_1', 'joint_name_2', ...]  # Replace with your joint names
    robot_state.joint_state.position = [0.0] * len(robot_state.joint_state.name)

    scene.attach_urdf_object(
        custom_end_effector_urdf,  # URDF of the custom end-effector
        robot.get_end_effector_link(),  # End-effector link name
        robot_state,
        end_effector_pose,
    )

    # Wait for a while or execute MoveIt! planning and execution here

    rospy.sleep(5.0)
