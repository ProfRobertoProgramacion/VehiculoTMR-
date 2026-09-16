#!/usr/bin/env python3
"""
Visualizador directo de la cámara USB física (/dev/video0) sin depender de ROS ni de Qt.
Ideal para verificar imagen, lente y enfoque al instante.
"""
import sys
import cv2

def main():
    dev_id = 0
    if len(sys.argv) > 1:
        dev_id = int(sys.argv[1])

    print(f"Abriendo camara /dev/video{dev_id}...")
    cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened():
        print(f"ERROR: No se pudo abrir /dev/video{dev_id}. Verifica conexion USB.")
        return

    print("Camara abierta con exito. Presiona 'q' en la ventana para salir.")
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Alerta: No se pudo leer cuadro.")
            break

        cv2.imshow("Camara USB TMR - Directa (Presiona 'q' para salir)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
