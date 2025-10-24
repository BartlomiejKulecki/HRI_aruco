import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray


class HRINode(Node):
    def __init__(self):
        super().__init__('hri_node')
        self.goal_publisher = self.create_publisher(Float32MultiArray, '/hri_node/joint_goal', 10)
        
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
            
            self.set_goal(center_x, center_y, img.shape)
      
        cv2.imshow('HRI_camera_image', img)
        cv2.waitKey(1)
            
        
    def set_goal(self, x, y, img_shape):    
        goal_msg = Float32MultiArray()
        goal = [1.0, -1.72, -2.2, -0.8, 1.6, -0.03]
        
        if x < img_shape[1] / 2:
            goal[0] += 0.5
            self.get_logger().info('Move Right')
        else:
            goal[0] -= 0.5
            self.get_logger().info('Move Left')
        
        if y < img_shape[0] / 2:
            self.get_logger().info('Move Forward')
            goal[1] -= 0.6
            goal[2] += 0.6
            goal[3] += 0.2

        goal_msg.data = goal
        self.goal_publisher.publish(goal_msg)

        

def main(args=None):
    rclpy.init(args=args)

    hri_node = HRINode()
    rclpy.spin(hri_node)

    hri_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()