#!/usr/bin/env python3
"""
Visualizador ligero en OpenCV para temas de ROS 2.
Compatible con QoS SensorDataQoS (Best Effort) y con refresco continuo
para evitar que GNOME marque la ventana como 'No responde'.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class RosCameraViewer(Node):
    def __init__(self, topic):
        super().__init__('ros_camera_viewer')
        self.topic = topic
        self.latest_frame = None
        self.frame_count = 0

        # Suscripcion con QoS SensorData (Best Effort) indispensable para camaras en ROS 2
        self.sub = self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            qos_profile_sensor_data)

        self.window_name = f"Camara ROS 2: {topic}"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Escuchando {topic} (QoS SensorData)... Esperando cuadros...")

        # Timer a 30 FPS para refrescar eventos de ventana (evita 'No responde')
        self.timer = self.create_timer(0.033, self.timer_callback)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.latest_frame = frame
                self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Error decodificando imagen: {e}")

    def timer_callback(self):
        if self.latest_frame is not None:
            cv2.imshow(self.window_name, self.latest_frame)
        else:
            # Pantalla de espera animada mientras llega la primera imagen
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Conectado a ROS 2", (180, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(placeholder, f"Esperando frames en: {self.topic}...", (60, 250),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
            cv2.putText(placeholder, "Inicia el nodo de la camara en la otra terminal", (80, 300),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            cv2.imshow(self.window_name, placeholder)

        # Clave: procesa eventos de X11 en cada ciclo para que la ventana nunca se congele
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            rclpy.shutdown()

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
