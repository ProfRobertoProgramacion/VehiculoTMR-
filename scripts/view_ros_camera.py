#!/usr/bin/env python3
"""
Visualizador ligero en OpenCV para temas de ROS 2.
NO utiliza PyQt ni Qt5 GUI, evitando el error: 'undefined symbol: glGenerateMipmap'.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class RosCameraViewer(Node):
    def __init__(self, topic):
        super().__init__('ros_camera_viewer')
        self.sub = self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            10)
        self.window_name = f"ROS 2: {topic} (Presiona 'q' para salir)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Escuchando {topic}... Esperando cuadros...")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow(self.window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main():
    topic = "/camera/image_raw/compressed"
    if len(sys.argv) > 1:
        topic = sys.argv[1]

    rclpy.init()
    node = RosCameraViewer(topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
