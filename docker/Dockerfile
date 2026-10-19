FROM ros:noetic-ros-base-focal
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace/swarm_tracking_ws
COPY scripts/install_dependencies.sh /tmp/install_dependencies.sh
RUN bash /tmp/install_dependencies.sh && rm -rf /var/lib/apt/lists/*
COPY . .
RUN bash scripts/build.sh
CMD ["bash"]
