# syntax=docker/dockerfile:1
# ============================================================================
# patrol-drone — ingest container (docset 05-logging-replay, M8 / T8.6, SWM-78)
# ----------------------------------------------------------------------------
# The DGX-side ingest service + queryable SQLite manifest (design §4.2.4, §4.4.3). Indexes each
# bag the upload daemon lands: derives duration + per-topic counts from the bag via `ros2 bag info`
# (so it needs a ROS env), reads identity from the sidecar, and upserts one manifest row.
#
# Deliberately small vs docker/sim & docker/dev: no PX4, no Gazebo, no build-from-source — just the
# ROS base (for `ros2 bag info` + the MCAP storage plugin) + this package's pure-Python code. The
# ROS distro is pinned from the canonical stack-manifest.toml (ADR-0004) at build time; the ARG has
# no default so a forgotten build arg fails loudly rather than silently picking a distro.
#
# Build (from repo root, so the COPY context includes docker/ingest/):
#   docker build -f docker/ingest/Dockerfile --build-arg ROS_DISTRO=jazzy -t patrol-ingest .
# Run (watch a landing dir, query the manifest):
#   docker run --rm -v /data/bags:/data/bags -v /data/manifest:/data/manifest patrol-ingest \
#     python -m ingest.manifest_query --recent 10 --db /data/manifest/bag_manifest.db
# ============================================================================

ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# The MCAP storage plugin so `ros2 bag info` can read the .mcap bags the recorder produces
# (sqlite is legacy; MCAP is the pinned bag format — plan M7 / design §6 constraint).
# hadolint ignore=DL3008
RUN apt-get update \
    && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ingest

# Pure-Python package — no build step. sqlite3 is stdlib; ros2 comes from the base.
COPY docker/ingest/ /opt/ingest/ingest/

ENV PYTHONPATH=/opt/ingest
ENV PATROL_MANIFEST_DB=/data/manifest/bag_manifest.db

# Default: print the manifest query help. Operators override the CMD with the watch loop or a query.
CMD ["python3", "-m", "ingest.manifest_query", "--help"]
