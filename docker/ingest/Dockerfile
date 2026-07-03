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
# Build (from repo root, so the COPY context includes docker/ingest/ + analysis/_shared/). The
# base digest + apt pins come from stack-manifest.toml [ingest] via gen_build_args.py --section ingest:
#   docker build -f docker/ingest/Dockerfile $(scripts/gen_build_args.py --section ingest) \
#     -t patrol-ingest .
# Run (watch a landing dir, query the manifest):
#   docker run --rm -v /data/bags:/data/bags -v /data/manifest:/data/manifest patrol-ingest \
#     python -m ingest.manifest_query --recent 10 --db /data/manifest/bag_manifest.db
# ============================================================================

# The base image is digest-pinned (ROS_BASE_DIGEST, stack-manifest.toml [ingest]) so the ingest image
# is reproducible — mirroring the digest-pin discipline the sim image gets. It is a DIFFERENT, smaller
# base than the sim/dev [container] one (plain ros:<distro>-ros-base), so it has its own [ingest]
# digest. Bump the manifest (re-resolve the digest / apt candidates), never edit inline, so drift is
# caught by manifest-drift CI. NB: no CI lane builds this Dockerfile — these pins are validated by a
# local/DGX build only (PR #16 / F-01).
ARG ROS_DISTRO
ARG ROS_BASE_DIGEST      # stack-manifest.toml [ingest] ros_base_digest (sha256:… for ros:<distro>-ros-base)
FROM ros:${ROS_DISTRO}-ros-base@${ROS_BASE_DIGEST}

# A pre-FROM ARG is only in scope for the FROM line; re-declare it (no default, so a forgotten
# build arg still fails loudly) to make ROS_DISTRO available to the RUN steps below.
ARG ROS_DISTRO

# `ros2 bag info` needs the rosbag2 CLI verb (rosbag2) + the MCAP storage plugin to read the .mcap
# bags the recorder produces (sqlite is legacy; MCAP is the pinned bag format — plan M7 / design §6).
# Both packages are version-pinned from the manifest for reproducibility; the two carry DIFFERENT apt
# candidates on the pinned base, so they take two ARGs (PR #16 / F-01).
ARG ROSBAG2_APT_VERSION       # stack-manifest.toml [ingest] rosbag2_apt_version
ARG ROSBAG2_MCAP_APT_VERSION  # stack-manifest.toml [ingest] rosbag2_mcap_apt_version
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        "ros-${ROS_DISTRO}-rosbag2=${ROSBAG2_APT_VERSION}" \
        "ros-${ROS_DISTRO}-rosbag2-storage-mcap=${ROSBAG2_MCAP_APT_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ingest

# Pure-Python package — no build step. sqlite3 is stdlib; ros2 comes from the base.
# _shared/ carries the neutral top-level module ingest.bounded_seen re-exports from (PR #16 / F-03) —
# COPY'd alongside so `from _shared.bounded_seen import …` resolves on /opt/ingest (already on PYTHONPATH).
COPY docker/ingest/ /opt/ingest/ingest/
COPY analysis/_shared/ /opt/ingest/_shared/

ENV PYTHONPATH=/opt/ingest
ENV PATROL_MANIFEST_DB=/data/manifest/bag_manifest.db

# Default: print the manifest query help. Operators override the CMD with the watch loop or a query.
CMD ["python3", "-m", "ingest.manifest_query", "--help"]
