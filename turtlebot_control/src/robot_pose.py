import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

def get_current_pose(data):
    current_pose.pose.pose.position.x = data.pose.pose.position.x
    current_pose.pose.pose.position.y = data.pose.pose.position.y
    current_pose.pose.pose.position.z = data.pose.pose.position.z
    current_pose.pose.pose.orientation.x = data.pose.pose.orientation.x
    current_pose.pose.pose.orientation.y = data.pose.pose.orientation.y
    current_pose.pose.pose.orientation.z = data.pose.pose.orientation.z
    current_pose.pose.pose.orientation.w = data.pose.pose.orientation.w

def convert(x,y,z,w):

    orientation_list = [x,y,z,w]
    (roll, pitch, yaw) = np.rad2deg(euler_from_quaternion (orientation_list))
    return roll, pitch, yaw

if __name__ == "__main__":
    rospy.init_node("robot_pose")
    global current_pose, robot_pose
    robot_pose = Quaternion()
    current_pose = PoseWithCovarianceStamped()
    goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    robotpose = rospy.Publisher("/robot/pose", Quaternion, queue_size=10)
    current = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, get_current_pose)
    x = float(input("x_coordinate of the desired position: "))
    y = float(input("y_coordinate of the desired position: "))
    x_quaternion = float(input("x_quat of the desired position: "))
    y_quaternion = float(input("y_quat of the desired position: "))
    z_quaternion = float(input("z_quat of the desired position: "))
    w_quaternion = float(input("w_quat of the desired position: "))
    x_initial = current_pose.pose.pose.position.x
    y_initial = current_pose.pose.pose.position.y
    r, p, yaw_initial = convert(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w)

    p = PoseStamped()
    p.header.frame_id='map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0
    p.pose.orientation.x = x_quaternion
    p.pose.orientation.y = y_quaternion
    p.pose.orientation.z = z_quaternion
    p.pose.orientation.w = 1

    goal.publish(p)
    while not rospy.is_shutdown():
        r, p, yaw = convert(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w)
        robot_pose.x = current_pose.pose.pose.position.x - x_initial
        robot_pose.y = current_pose.pose.pose.position.y - y_initial
        robot_pose.z = 0
        robot_pose.w = yaw - yaw_initial
        robotpose.publish(robot_pose)

