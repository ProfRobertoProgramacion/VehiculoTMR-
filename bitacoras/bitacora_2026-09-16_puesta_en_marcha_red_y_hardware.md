# 📋 Bitácora de Desarrollo - Vehículo Autónomo TMR
**Fecha:** 16 de Septiembre de 2026  
**Autor:** Roberto Sotelo (`ProfRobertoProgramacion`) & Antigravity AI  
**Repositorio GitHub:** [https://github.com/ProfRobertoProgramacion/VehiculoTMR-.git](https://github.com/ProfRobertoProgramacion/VehiculoTMR-.git) (Público)  

---

## 🎯 1. Resumen y Contexto del Proyecto

Desarrollo de un vehículo autónomo a escala con dirección tipo Ackermann para competir en la pista oficial del **Torneo Mexicano de Robótica (TMR)**.

### Stack Tecnológico:
* **A bordo del vehículo:** Odroid M1S (SoC Rockchip ARM64, Ubuntu 20.04 Focal, ROS 2 Foxy).
* **Cámara a bordo:** Cámara USB Logitech en `/dev/video0`.
* **Microcontrolador / Actuadores:** Arduino Nano (chip CH340 en `/dev/ttyUSB0`) controlando Servo de dirección y ESC para motor de tracción mediante firmware con Watchdog (`firmware/arduino_controller/arduino_controller.ino`).
* **Estación de Desarrollo / Procesamiento Remoto:** PC con Windows 11 y WSL2 (Ubuntu 20.04, ROS 2 Foxy, OpenCV 4.2 C++ y Python).
* **Ecosistema de Red:** ROS 2 DDS sobre FastDDS con enlace Unicast punto a punto sobre Wi-Fi.

---

## 🌐 2. Topología de Red y Dispositivos

| Dispositivo | Hostname | Usuario | Dirección IP | Función |
| :--- | :--- | :--- | :--- | :--- |
| **Odroid M1S** | `gnome-desktop` | `odroid` | `192.168.0.181` | Ejecución a bordo: captura de cámara USB y puente serie hacia Arduino |
| **PC Windows 11 (WSL2)** | `sotelo` | `roberto` | `192.168.0.236` | Procesamiento pesado: Visión OpenCV C++ (IPM y carril), Controlador Stanley, depuración |

### Variables de Entorno ROS 2 Cruciales:
* `ROS_DOMAIN_ID=42`: Debe ser exactamente el mismo en Odroid y PC para aislar el tráfico DDS en la red local.
* `ROS_LOCALHOST_ONLY=0`: Obligatorio en ambos para que ROS 2 no se restrinja a la interfaz `127.0.0.1` y permita salida por la antena Wi-Fi / Ethernet.

---

## 🛠️ 3. Problemas Detectados y Soluciones Aplicadas

### ❌ Problema 1: Configuración de Git en WSL y Error `not in a git directory`
* **Síntoma:** Al configurar el usuario con `git config user.name ...` salía `fatal: not in a git directory`.
* **Causa:** El comando se ejecutaba en la carpeta personal de Linux (`/home/roberto`) en lugar de estar dentro de `/mnt/c/Users/rober/OneDrive/Escritorio/VehiculoTMR`.
* **Solución:** Navegar al directorio raíz del proyecto o utilizar la bandera global `git config --global user.name "ProfRobertoProgramacion"` y `git config --global user.email "roberto_sotelo@outlook.es"`.

---

### ❌ Problema 2: Error `%0D` al Clonar en Odroid (`Carriage Return`)
* **Síntoma:** Al clonar en la Odroid salía `fatal: credential value for username contains carriage return ... Password for 'https://%0DProfRobertoProgramacion@github.com'`.
* **Causa:** 
  1. El repositorio era Privado y requería credenciales.
  2. Al copiar y pegar el usuario desde Windows hacia la terminal de Linux (por SSH o VNC), se coló un salto de línea estilo Windows (`\r` en ASCII hex `%0D`), el cual Git bloquea por seguridad.
* **Solución:** Se convirtió el repositorio de GitHub a **Público** (`https://github.com/ProfRobertoProgramacion/VehiculoTMR-`). De esta forma, la Odroid no requiere usuarios, contraseñas ni tokens para hacer `git clone` o `git pull`.

---

### ❌ Problema 3: Puerto Serie del Arduino Erróneo (`/dev/ttyACM0` vs `/dev/ttyUSB0`)
* **Síntoma:** El nodo `arduino_bridge_node` mostraba: `[WARN] No se puede abrir el puerto /dev/ttyACM0. Reintentando...`.
* **Causa:** El código venía por defecto configurado para `/dev/ttyACM0` (típico de Arduino Uno oficial), pero en la Odroid el microcontrolador conectado es un Arduino Nano con chip conversor USB-Serial CH340, el cual se reconoce como `/dev/ttyUSB0`.
* **Solución:**
  1. Se actualizó el valor por defecto en `tmr_bringup/config/vehicle_params.yaml` a `/dev/ttyUSB0`.
  2. Se añadió en `tmr_hardware/src/arduino_bridge_node.cpp` un algoritmo de **auto-detección y tolerancia a fallos**: si el puerto principal no responde, prueba dinámicamente `/dev/ttyUSB0`, `/dev/ttyACM0`, etc.
  3. Se recordó conceder permisos de acceso al puerto en Linux: `sudo usermod -aG dialout $USER` o `sudo chmod 666 /dev/ttyUSB0`.

---

### ❌ Problema 4: Excepción Fatal de Tiempo en ROS 2 (`can't subtract times [1 != 2]`)
* **Síntoma:** `arduino_bridge_node` crasheaba a los 100ms de iniciar con:
  `terminate called after throwing an instance of 'std::runtime_error'`  
  `what(): can't subtract times with different time sources [1 != 2]`
* **Causa:** En `arduino_bridge_node.cpp`, la variable `rclcpp::Time last_cmd_time_` no fue inicializada en el constructor. Por defecto en C++ tomaba fuente `RCL_SYSTEM_TIME` (tipo 1), mientras que en `heartbeat()` se llamaba a `this->now()`, el cual devuelve `RCL_ROS_TIME` (tipo 2). En ROS 2 no se pueden restar dos timestamps con fuentes de reloj distintas.
* **Solución:** Se agregó en el constructor de `ArduinoBridgeNode`:
  ```cpp
  last_cmd_time_ = this->now();
  ```

---

### ❌ Problema 5: Error de Qt5 en Odroid (`undefined symbol: glGenerateMipmap`)
* **Síntoma:** Al intentar ver la cámara en la Odroid con `rqt_image_view` salía:
  `ImportError: /lib/aarch64-linux-gnu/libQt5Gui.so.5: undefined symbol: glGenerateMipmap`
* **Causa:** La Odroid M1S tiene arquitectura ARM64 con GPU Mali. Los controladores OpenGL ES de Mali colisionan con las llamadas de escritorio que hace PyQt5 en Ubuntu.
* **Solución:** Se crearon dos visualizadores nativos en OpenCV sin depender de Qt:
  * `scripts/test_camera_direct.py`: Prueba directa de `/dev/video0` con OpenCV (abre ventana en 1 segundo).
  * `scripts/view_ros_camera.py`: Nodo subscriptor de ROS 2 en Python que decodifica `/camera/image_raw/compressed` y lo muestra con `cv2.imshow`.

---

### ❌ Problema 6: Incompatibilidad de QoS y Ventana "No Responde" en OpenCV
* **Síntoma:** Al abrir `scripts/view_ros_camera.py`, la ventana quedaba negra y GNOME mostraba la alerta "La ventana no responde".
* **Causa:** 
  1. `usb_camera_node` publica con `SensorDataQoS` (`RELIABILITY_BEST_EFFORT`), mientras que el subscriptor en Python se suscribió con el default (`RELIABILITY_RELIABLE`). En ROS 2, **un subscriptor Reliable nunca recibe mensajes de un publicador Best Effort** (se descartan silenciosamente).
  2. Como no llegaban mensajes, no se ejecutaba `cv2.waitKey()`, impidiendo que el event loop de X11 respondiera al gestor de ventanas de GNOME.
* **Solución:** 
  1. Se configuró el subscriptor con `qos_profile_sensor_data`.
  2. Se añadió un temporizador a 30 FPS (`self.create_timer(0.033, self.timer_callback)`) con un placeholder visual que bombea `cv2.waitKey(1)` continuamente.

---

### ❌ Problema 7: Bloqueo de Red Wi-Fi (Routers & Firewall de Windows)
* **Síntoma:** En la Odroid la cámara corría bien, pero en la PC `ros2 topic info /camera/image_raw/compressed` reportaba `Publisher count: 0`.
* **Causa:** 
  1. **Router Wi-Fi:** Los módems residenciales activan por defecto filtrado/bloqueo de paquetes UDP Multicast inalámbricos. El descubrimiento estándar de ROS 2 depende de Multicast.
  2. **Firewall de Windows:** La conexión de red en Windows estaba clasificada como `Public`, bloqueando todo tráfico UDP entrante hacia WSL2.
* **Solución:** 
  1. Se creó el archivo de configuración Unicast `config/fastdds_unicast.xml` apuntando directamente a las IPs fijas (`192.168.0.181` y `192.168.0.236`).
  2. Se integró en `scripts/setup_ros_network.sh` la variable `FASTRTPS_DEFAULT_PROFILES_FILE`.
  3. Se creó como alternativa un servidor HTTP en streaming `scripts/web_camera_server.py` para visualizar en Chrome/Edge en el puerto 8080.
  4. Se configuró el Firewall de Windows / Red Privada para autorizar los puertos UDP de ROS 2.

---

## 🚀 4. Guía de Ejecución Rápida para Próximas Sesiones

### A) En la Odroid M1S (A bordo del auto)
```bash
cd ~/VehiculoTMR
source scripts/setup_ros_network.sh

# Modo 1: Solo Cámara
ros2 run tmr_camera usb_camera_node

# Modo 2: Cámara + Arduino (Modo Mixto Odroid + PC)
ros2 launch tmr_bringup odroid_vehicle.launch.py

# Modo 3: 100% Autónomo en Pista (Todo adentro de la Odroid)
ros2 launch tmr_bringup all_on_board.launch.py
```

### B) En la PC (Windows 11 / WSL2)
```bash
cd /mnt/c/Users/rober/OneDrive/Escritorio/VehiculoTMR
source scripts/setup_ros_network.sh

# Ver cámara en vivo desde PC:
python3 scripts/view_ros_camera.py

# Lanzar Procesamiento de Visión C++ y Control Stanley en PC:
ros2 launch tmr_bringup pc_remote_processing.launch.py

# Ver imagen de depuración de carril en tiempo real:
ros2 run rqt_image_view rqt_image_view /tmr/vision_debug
```

### C) Sincronización con GitHub
* **Para subir cambios desde la PC:**
  ```bash
  git add .
  git commit -m "Descripción del cambio"
  git push origin main
  ```
* **Para actualizar en la Odroid:**
  ```bash
  cd ~/VehiculoTMR
  git pull
  cd tmr_ws && colcon build --symlink-install && source install/setup.bash
  ```

---
*Fin del registro de bitácora.*
