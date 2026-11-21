# syntax=docker/dockerfile:1
FROM python:3.10-slim AS g1-model

ARG UNITREE_MUJOCO_VERSION=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl git \
    && rm -rf /var/lib/apt/lists/*
RUN git init /tmp/unitree \
    && git -C /tmp/unitree remote add origin https://github.com/unitreerobotics/unitree_mujoco.git \
    && git -C /tmp/unitree sparse-checkout set unitree_robots/g1 \
    && git -C /tmp/unitree fetch --depth 1 --filter=blob:none origin "$UNITREE_MUJOCO_VERSION" \
    && git -C /tmp/unitree checkout --detach FETCH_HEAD

FROM ubuntu:22.04 AS arm-sim

ARG UNITREE_PYTHON_VERSION=814556d15970dd2ecf1c9984e845ca02ab07e206
ENV PATH=/opt/venv/bin:$PATH \
    CYCLONEDDS_HOME=/opt/cyclonedds \
    LD_LIBRARY_PATH=/opt/cyclonedds/lib \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates cmake git libgl1 libglib2.0-0 \
    python3-dev python3-venv && rm -rf /var/lib/apt/lists/*

# Cyclone DDS 0.10.2 has no Linux ARM64 wheel on PyPI.
RUN git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds \
    && cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CYCLONEDDS_HOME" \
       -DENABLE_SHM=OFF -DENABLE_SSL=OFF -DBUILD_TESTING=OFF \
    && cmake --build /tmp/cyclonedds/build --target install -j4 \
    && rm -rf /tmp/cyclonedds

RUN git init /opt/unitree_sdk2_python \
    && git -C /opt/unitree_sdk2_python fetch --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git "$UNITREE_PYTHON_VERSION" \
    && git -C /opt/unitree_sdk2_python checkout --detach FETCH_HEAD \
    && python3 -m venv /opt/venv \
    && pip install --no-cache-dir -e /opt/unitree_sdk2_python mujoco==3.3.6

COPY --from=g1-model /tmp/unitree/unitree_robots/g1 /opt/unitree_mujoco/unitree_robots/g1
COPY --from=g1-model /tmp/unitree/LICENSE /opt/unitree_mujoco/LICENSE
WORKDIR /work

FROM g1-model AS web-assets
ARG THREE_VERSION=0.180.0
RUN mkdir -p /opt/web-assets/vendor /tmp/three \
    && curl --fail --silent --show-error --location "https://registry.npmjs.org/three/-/three-${THREE_VERSION}.tgz" \
       --output /tmp/three/three.tgz \
    && tar -xzf /tmp/three/three.tgz -C /tmp/three \
    && cp /tmp/three/package/build/three.module.min.js /tmp/three/package/build/three.core.min.js \
       /tmp/three/package/examples/jsm/loaders/STLLoader.js \
       /tmp/three/package/examples/jsm/controls/OrbitControls.js /opt/web-assets/vendor/ \
    && cp /tmp/three/package/LICENSE /opt/web-assets/vendor/LICENSE-three.txt \
    && cp /tmp/unitree/LICENSE /opt/web-assets/LICENSE-Unitree.txt
COPY g1_control/export_model.py /tmp/export_model.py
RUN python /tmp/export_model.py /tmp/unitree/unitree_robots/g1/g1_29dof.xml /opt/web-assets

FROM python:3.10-slim AS mock
WORKDIR /work
COPY requirements-web.txt /tmp/requirements-web.txt
RUN pip install --no-cache-dir -r /tmp/requirements-web.txt
COPY --from=web-assets /opt/web-assets /opt/web-assets

FROM arm-sim AS web-sim
COPY requirements-web.txt /tmp/requirements-web.txt
RUN pip install --no-cache-dir -r /tmp/requirements-web.txt
COPY --from=web-assets /opt/web-assets /opt/web-assets

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libgl1-mesa-dri libglx-mesa0 libx11-6 libxcursor1 libxi6 libxinerama1 libxrandr2 \
    novnc websockify x11vnc xauth xvfb && rm -rf /var/lib/apt/lists/*
