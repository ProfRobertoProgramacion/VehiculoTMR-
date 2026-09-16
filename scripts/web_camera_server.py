#!/usr/bin/env python3
"""
Servidor Web de Video para Odroid M1S (VehiculoTMR)
Sirve el stream de /camera/image_raw/compressed directamente por HTTP.
Permite ver la camara en vivo desde cualquier navegador (Chrome, Edge) en Windows
saltandose por completo bloqueos de firewall, problemas de DDS y de red.
"""
import sys
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

latest_jpeg = None
frame_lock = threading.Lock()
frame_count = 0

HTML_PAGE = """<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <title>Vehiculo TMR - Camara en Vivo</title>
    <style>
        body {
            background-color: #0d1117;
            color: #c9d1d9;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            margin: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
        }
        .container {
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 12px;
            padding: 24px;
            box-shadow: 0 8px 24px rgba(0,0,0,0.5);
            text-align: center;
            max-width: 720px;
        }
        h1 {
            color: #58a6ff;
            margin-top: 0;
            font-size: 1.6rem;
        }
        .stream-box {
            border: 2px solid #238636;
            border-radius: 8px;
            overflow: hidden;
            background: #000;
            display: inline-block;
        }
        img {
            display: block;
            max-width: 100%;
            height: auto;
        }
        .badge {
            display: inline-block;
            background: #238636;
            color: white;
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 0.85rem;
            font-weight: bold;
            margin-bottom: 15px;
        }
        .footer {
            margin-top: 15px;
            font-size: 0.85rem;
            color: #8b949e;
        }
    </style>
</head>
<body>
    <div class="container">
        <span class="badge">EN VIVO - ROS 2</span>
        <h1>🏎️ Vehiculo TMR - Camara Odroid M1S</h1>
        <div class="stream-box">
            <img src="/stream.mjpg" alt="Transmision de Camara" width="640" height="480">
        </div>
        <div class="footer">
            Canal: <code>/camera/image_raw/compressed</code> &bull; Resolucion: 640x480 &bull; Odroid M1S
        </div>
    </div>
</body>
</html>
"""

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Desactivar logs ruidosos de HTTP por cada frame
        return

    def do_GET(self):
        global latest_jpeg, frame_lock

        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
            return

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                while True:
                    with frame_lock:
                        data = latest_jpeg

                    if data is not None:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')

                    threading.Event().wait(0.033) # ~30 FPS
            except (ConnectionResetError, BrokenPipeError):
                return
        else:
            self.send_response(404)
            self.end_headers()

class WebCameraNode(Node):
    def __init__(self):
        super().__init__('web_camera_server')
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data)
        self.get_logger().info("Servidor Web suscrito a /camera/image_raw/compressed")

    def image_callback(self, msg):
        global latest_jpeg, frame_lock, frame_count
        with frame_lock:
            latest_jpeg = bytes(msg.data)
            frame_count += 1
            if frame_count == 1:
                self.get_logger().info("¡Primer frame recibido! Transmision web lista.")

def run_http_server(port=8080):
    server = HTTPServer(('0.0.0.0', port), StreamHandler)
    print(f"\n=======================================================")
    print(f" Servidor Web de Camara Activo!")
    print(f" Abre en tu PC (Chrome o Edge): http://localhost:{port}")
    print(f" O desde la red: http://192.168.0.181:{port}")
    print(f"=======================================================\n")
    server.serve_forever()

def main():
    port = 8080
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    rclpy.init()
    node = WebCameraNode()

    # Servidor HTTP en un hilo secundario
    http_thread = threading.Thread(target=run_http_server, args=(port,), daemon=True)
    http_thread.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
