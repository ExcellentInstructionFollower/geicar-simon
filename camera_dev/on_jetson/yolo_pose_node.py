#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloPoseNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_node')
        
        # Inférence sur GPU CUDA (device=0)
        # Pour des performances maximales sur Jetson, utilisez un export TensorRT (.engine)
        self.model = YOLO('weights.pt') 
        self.bridge = CvBridge()

        # Souscription au flux vidéo brut de usb_cam
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # Publication du flux annoté pour web_video_server
        self.pub = self.create_publisher(Image, '/image_pose', 10)
        self.get_logger().info("Nœud YOLO Pose initialisé et prêt sur GPU.")

    def image_callback(self, msg):
        # Conversion du message ROS2 Image en frame OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inférence YOLO
        results = self.model.predict(source=cv_img, device=0, verbose=False, conf=0.5)
        annotated_frame = results[0].plot()

        # Publication de l'image annotée
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()