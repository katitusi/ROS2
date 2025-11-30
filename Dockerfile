# ============================================
# Multi-stage Dockerfile for ROS2 Humble
# With GPU support, CycloneDDS, and workspace
# ============================================

# ---- Stage 1: Base ROS2 image ----
FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install essential ROS2 packages and tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    git \
    wget \
    curl \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ---- Stage 2: Development image ----
FROM base AS dev

# Install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gdb \
    ccache \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Setup ccache for faster rebuilds
ENV PATH="/usr/lib/ccache:$PATH"

WORKDIR /ws

# Copy CycloneDDS configuration
COPY cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
if [ -f /ws/install/setup.bash ]; then\n\
    source /ws/install/setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# ---- Stage 3: Build workspace ----
FROM dev AS builder

# Copy source files
COPY src ./src

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install

# ---- Stage 4: Runtime image (smaller) ----
FROM base AS runtime

WORKDIR /ws

# Copy built artifacts from builder
COPY --from=builder /ws/install ./install
COPY cyclonedds.xml /etc/cyclonedds.xml

ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Create entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /ws/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
