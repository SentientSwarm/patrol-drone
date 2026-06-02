# scripts — Utility scripts

One-off and operational scripts. Setup helpers, dev environment bootstrappers, bag manipulation tools, ad-hoc utilities.

Anything substantial — that needs tests, lives more than one phase, or is invoked by other code — should be a proper Python package under `ros2_ws/src/`, not here.

Examples of what belongs here:
- Bash setup scripts for installing host dependencies
- One-off bag conversion or trimming utilities
- Operational helpers (start agent, check ports, etc.)
