FROM ubuntu:22.04

ARG SDK_VERSION=b7fec6020eb93d4c059a6edf19905e12a75a8d3d
ARG UNITREE_MUJOCO_VERSION=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates libyaml-cpp-dev libeigen3-dev \
    libboost-program-options-dev libfmt-dev libspdlog-dev libglfw3-dev libgl1-mesa-dri \
    libgl1-mesa-dev xvfb xauth && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-aarch64.tar.gz \
    | tar -xz -C /opt && ln -s /opt/mujoco-3.3.6 /opt/mujoco
ENV LD_LIBRARY_PATH=/opt/mujoco/lib
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /opt/unitree_sdk2 \
    && git -C /opt/unitree_sdk2 checkout --detach "$SDK_VERSION" \
    && cmake -S /opt/unitree_sdk2 -B /tmp/sdk-build -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
    && cmake --build /tmp/sdk-build -j4 && cmake --install /tmp/sdk-build \
    && rm -rf /tmp/sdk-build
RUN git clone https://github.com/unitreerobotics/unitree_mujoco.git /opt/unitree_mujoco \
    && git -C /opt/unitree_mujoco checkout --detach "$UNITREE_MUJOCO_VERSION" \
    && ln -s /opt/mujoco /opt/unitree_mujoco/simulate/mujoco \
    && cmake -S /opt/unitree_mujoco/simulate -B /opt/unitree_mujoco/simulate/build \
    && cmake --build /opt/unitree_mujoco/simulate/build -j4
WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11vnc novnc websockify && rm -rf /var/lib/apt/lists/*
CMD ["bash"]
