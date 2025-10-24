import rclpy
from rclpy.node import Node
import cv2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


class HRINode(Node):
    def __init__(self):
        super().__init__('hri_node')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info(f"Waiting for action server on /scaled_joint_trajectory_controller/follow_joint_trajectory")
        self._action_client.wait_for_server()
        self._send_goal_future = None
        self._get_result_future = None
        self.goal = None
        self.executing_trajectory = False 
        
        self.br = CvBridge()
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
               

    def image_callback(self, msg):        
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        
        # Create the ArUco detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            corner = corners[0][0]
            # draw center of the marker
            center_x = int((corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4)
            center_y = int((corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4)
            cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            
            if not self.executing_trajectory or self.goal is None:
                self.set_goal(center_x, center_y, img.shape)
                self._send_goal_future = self._action_client.send_goal_async(self.goal)
                self._send_goal_future.add_done_callback(self.goal_response_callback)
                self.get_logger().info('Publishing goal')
                self.executing_trajectory = True
            
        cv2.imshow('HRI_camera_image', img)
        cv2.waitKey(1)
            
        
    def set_goal(self, x, y, img_shape):    
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [1.0, -1.72, -2.2, -0.8, 1.6, -0.03]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        if x < img_shape[1] / 2:
            point.positions[0] += 0.5
            self.get_logger().info('Move Right')
        else:
            point.positions[0] -= 0.5
            self.get_logger().info('Move Left')
        
        if y < img_shape[0] / 2:
            self.get_logger().info('Move Forward')
            point.positions[1] -= 0.6
            point.positions[2] += 0.6
            point.positions[3] += 0.2

        traj_msg.points.append(point)
        self.goal = FollowJointTrajectory.Goal()
        self.goal.trajectory = traj_msg
        self.goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        self.goal.goal_tolerance = [JointTolerance(joint_name=name, position=0.05, velocity=0.1, acceleration=0.1) for name in traj_msg.joint_names]

       
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.executing_trajectory = False
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            raise RuntimeError("Executing trajectory failed. " + result.error_string)
        
        
    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"
         


def main(args=None):
    rclpy.init(args=args)

    hri_node = HRINode()
    rclpy.spin(hri_node)

    hri_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()