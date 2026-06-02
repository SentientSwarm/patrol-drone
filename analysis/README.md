# analysis — Bag analysis and exploration

Scripts, Jupyter notebooks, and one-off analysis tools that operate on recorded rosbags (MCAP format).

This is where exploratory data analysis lives — what perception saw, how the mission state machine behaved, where VIO drifted. **Not** for production code; that goes in `ros2_ws/src/`.

## Conventions

- Use `mcap` Python library to read bags directly, or `rosbag2_py` for ROS-aware access.
- Notebooks are reviewable artifacts — strip outputs before committing (`nbstripout` or similar pre-commit hook).
- Long-running analysis (e.g., training data prep for Phase 6) belongs in DGX-side pipelines, not here.
