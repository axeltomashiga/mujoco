#!/usr/bin/env bash
# ==============================================================================
# Script de configuracion para WSL2 / Ubuntu (ROS 2 Jazzy / Humble + MuJoCo)
# ==============================================================================

set -e

# 1. Validar que se esta ejecutando dentro de Linux / WSL y no en Windows (Git Bash/MSYS)
OS_TYPE="$(uname -s 2>/dev/null || echo 'Unknown')"
if [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin"* || "$OS_TYPE" != "Linux" ]]; then
    echo "=========================================================================="
    echo " [ERROR] Este script debe ejecutarse DENTRO de Ubuntu en WSL2."
    echo " Parece que lo ejecutaste desde una terminal de Windows (Git Bash / MSYS)."
    echo "=========================================================================="
    echo ""
    echo " Para ejecutarlo correctamente en WSL2, usa cualquiera de estas dos opciones:"
    echo ""
    echo "  Opcion A (Desde PowerShell en Windows):"
    echo "    wsl -d Ubuntu -e bash -c \"cd /mnt/d/proyectAgy/mujoco && ./setup_wsl2.sh\""
    echo ""
    echo "  Opcion B (Entrando a WSL):"
    echo "    1. Abre la terminal de Ubuntu: wsl -d Ubuntu"
    echo "    2. Entra a la carpeta: cd /mnt/d/proyectAgy/mujoco"
    echo "    3. Ejecuta: ./setup_wsl2.sh"
    echo ""
    exit 1
fi

echo "=== Iniciando configuracion en WSL2 (Linux) ==="

# 2. Detectar distribucion de Ubuntu leyendo /etc/os-release directamente
UBUNTU_CODENAME="noble"
if [ -f /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    UBUNTU_CODENAME="${UBUNTU_CODENAME:-$VERSION_CODENAME}"
fi

if [ -z "$UBUNTU_CODENAME" ]; then
    UBUNTU_CODENAME="noble"
fi

echo "Distribucion Ubuntu detectada: $UBUNTU_CODENAME"

# 3. Determinar distribucion de ROS 2 correspondiente
if [ "$UBUNTU_CODENAME" = "noble" ] || [ "$UBUNTU_CODENAME" = "24.04" ]; then
    TARGET_ROS="jazzy"
else
    TARGET_ROS="humble"
fi

# 4. Verificar o instalar ROS 2
if [ -d "/opt/ros/jazzy" ]; then
    ROS_DISTRO="jazzy"
    echo "ROS 2 Jazzy detectado en /opt/ros/jazzy."
elif [ -d "/opt/ros/humble" ]; then
    ROS_DISTRO="humble"
    echo "ROS 2 Humble detectado en /opt/ros/humble."
else
    echo "ROS 2 no instalado en /opt/ros. Instalando ROS 2 $TARGET_ROS..."
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl gnupg
    sudo add-apt-repository -y universe

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y "ros-${TARGET_ROS}-ros-base" "ros-${TARGET_ROS}-rmw-cyclonedds-cpp" "ros-${TARGET_ROS}-rmw-fastrtps-cpp" python3-colcon-common-extensions
    ROS_DISTRO="$TARGET_ROS"
fi

echo "Usando ROS_DISTRO: $ROS_DISTRO"

# 5. Instalar dependencias de Python y graficos OpenGL para MuJoCo
echo "Instalando dependencias de Python y GLFW para MuJoCo..."
sudo apt-get install -y python3-pip python3-colcon-common-extensions libglfw3 libglfw3-dev

pip3 install --break-system-packages mujoco numpy 2>/dev/null || pip3 install mujoco numpy

# 6. Compilar el paquete de ROS 2 con colcon
echo "Compilando paquete mujoco_ros2_sim..."
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install

# 7. Mensaje final con instrucciones
WS_DIR="$(pwd)"
echo ""
echo "=========================================================================="
echo "           Entorno MuJoCo + ROS 2 + DDS configurado con exito             "
echo "=========================================================================="
echo "Para correr la simulacion del Unitree G1 ejecuta en tu terminal de WSL:"
echo ""
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WS_DIR}/install/setup.bash"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  export CYCLONEDDS_URI=file://${WS_DIR}/config/cyclonedds.xml"
echo "  export ROS_DOMAIN_ID=42"
echo ""
echo "  ros2 launch mujoco_ros2_sim full_demo.launch.py"
echo "=========================================================================="
