FROM osrf/ros:jazzy-desktop

# Variables de entorno
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Instalar dependencias del sistema y graficas para MuJoCo
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-fastrtps-cpp \
    libgl1 \
    libglx-mesa0 \
    libglfw3 \
    libglfw3-dev \
    libglew-dev \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Instalar MuJoCo en Python
RUN pip3 install --no-cache-dir --break-system-packages mujoco numpy

# Crear workspace
WORKDIR /ros2_ws
COPY src /ros2_ws/src
COPY config /ros2_ws/config
COPY models /ros2_ws/models

# Compilar workspace con colcon
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Script de entrada para cargar ROS 2 y variables DDS automaticamente
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///ros2_ws/config/cyclonedds.xml" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /root/.bashrc

CMD ["bash"]
