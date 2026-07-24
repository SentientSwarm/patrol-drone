# The pre-built, digest-pinned replay-lane CI image (PR #16, Mira Medium F-06 / review 4728294643).
#
# The replay-regression lane previously apt-installed its runtime (git-lfs + the ROS runtime +
# rsync) at JOB TIME from mutable package indexes — unpinned, and re-raised across two review
# cycles. This image bakes that exact runtime ONCE at image-build time; the lane then references
# the published image by immutable digest (container.image: ghcr.io/...@sha256:...), making the
# lane's entire toolchain immutable — "a reviewed image with the required tools" (the review's own
# stated option). Deliberately NOT `apt-get install pkg=ver` pins: packages.ros.org has no snapshot
# service and prunes superseded versions, so version pins rot into hard CI failures; the image
# freezes a known-good resolved set by digest instead. Also removes ~90 MB of per-run apt work.
#
# Build/publish/flip flow: README.md beside this file (the build-replay-lane-image workflow
# publishes to GHCR and prints the digest; .github/workflows/replay-regression.yml pins it).
#
# Same digest-pinned base the lane's `container:` uses today, so the flip changes nothing but
# where the runtime comes from.
FROM rostooling/setup-ros-docker:ubuntu-noble-latest@sha256:f0ba5d588664d0533f4bee2d1078e29d7ef151989fa0ba020e4ac935d38a7676

# Kept in sync with stack-manifest.toml middleware.ros_distro (the same pin replay-regression.yml
# sets as a job env; baked here so the image is self-describing).
ENV ROS_DISTRO=jazzy

# The lane's full runtime, resolved ONCE here (the job-time apt steps are deleted when the lane
# pins this image):
#   git-lfs                       — checkout smudge for the LFS reference bag
#   rclpy / std-msgs / sensor-msgs — the live subscriber counter (every asserted topic subscribed)
#   geometry-msgs / diagnostic-msgs / builtin-interfaces — patrol_interfaces msg deps
#   rosidl generators/runtime(+py) + ament-cmake — colcon-builds px4_msgs + patrol_interfaces and
#                                   resolves message classes by name (get_message, + python3-numpy)
#   rosbag2 (+ MCAP plugin)       — `ros2 bag play` / `info` on the .mcap reference bag
#   rsync                         — the stand-in test's RsyncSshTransport
# The dpkg-query manifest records the exact resolved versions this image froze, so the image is
# auditable ("a REVIEWED image"): docker run <img> cat /usr/local/share/patrol-drone/replay-lane-apt.manifest
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git-lfs \
        "ros-${ROS_DISTRO}-rclpy" \
        "ros-${ROS_DISTRO}-std-msgs" \
        "ros-${ROS_DISTRO}-sensor-msgs" \
        "ros-${ROS_DISTRO}-geometry-msgs" \
        "ros-${ROS_DISTRO}-diagnostic-msgs" \
        "ros-${ROS_DISTRO}-builtin-interfaces" \
        "ros-${ROS_DISTRO}-rosidl-default-generators" \
        "ros-${ROS_DISTRO}-rosidl-default-runtime" \
        "ros-${ROS_DISTRO}-rosidl-runtime-py" \
        "ros-${ROS_DISTRO}-ament-cmake" \
        "ros-${ROS_DISTRO}-rosbag2" \
        "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
        python3-numpy \
        rsync \
    && mkdir -p /usr/local/share/patrol-drone \
    && dpkg-query -W -f '${Package}=${Version}\n' \
        git-lfs rsync python3-numpy "ros-${ROS_DISTRO}-*" \
        > /usr/local/share/patrol-drone/replay-lane-apt.manifest \
    && rm -rf /var/lib/apt/lists/*
