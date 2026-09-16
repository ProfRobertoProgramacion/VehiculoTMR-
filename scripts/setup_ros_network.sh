#!/usr/bin/env bash
# ==============================================================================
# Script de Configuración de Red ROS 2 Foxy (Ejecutar con: source setup_ros_network.sh)
# Usar tanto en WSL2 como en la Odroid M1S
# ==============================================================================

echo "=== Configurando Entorno ROS 2 Foxy para Red Distribuida ==="

# 1. Cargar ROS 2 Foxy base si no está cargado
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo "[OK] ROS 2 Foxy base cargado (/opt/ros/foxy/setup.bash)"
    else
        echo "[ALERTA] No se encontró /opt/ros/foxy/setup.bash"
    fi
fi

# 2. Cargar el Workspace local si existe
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$SCRIPT_DIR/tmr_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/tmr_ws/install/setup.bash"
    echo "[OK] Workspace local cargado ($SCRIPT_DIR/tmr_ws/install/setup.bash)"
fi

# 3. Asignar Dominio DDS compartido (debe ser el mismo en PC y Odroid)
export ROS_DOMAIN_ID=42
echo "[OK] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 4. Asegurar que no esté restringido a localhost
export ROS_LOCALHOST_ONLY=0
echo "[OK] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

# 5. RMW Middleware por defecto (FastDDS o CycloneDDS)
# Si se tiene problemas con FastDDS por defecto en Wi-Fi, se puede instalar:
# sudo apt install ros-foxy-rmw-cyclonedds-cpp
# y descomentar la siguiente línea:
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "=== Entorno listo. Para verificar dispositivos en red ejecuta: ros2 node list ==="
