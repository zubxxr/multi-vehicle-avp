# ===============================
# AVP-CF (Built on DMAVA Autoware)
# ===============================

FROM zubxxr/dmava-autoware:universe-cuda-2026.1

ENV DEBIAN_FRONTEND=noninteractive

# Install only what AVP managers need (if missing)
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-shapely \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /avp_ws

# Copy only AVP managers source
COPY src ./src

# Build on top of Autoware environment
RUN /bin/bash -c " \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
"

ENTRYPOINT ["/bin/bash"]
