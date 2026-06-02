#!/usr/bin/env bash
# Push the local patrol-drone repo to github.com/SentientSwarm/patrol-drone
#
# Prerequisites:
#   - git installed
#   - One of:
#       (a) `gh` CLI installed and authenticated (`gh auth status` works), OR
#       (b) you create the empty repo via the GitHub web UI first
#
# Run from inside the unpacked patrol-drone/ directory.

set -euo pipefail

ORG="SentientSwarm"
REPO="patrol-drone"

if ! git rev-parse --git-dir > /dev/null 2>&1; then
  echo "Error: not inside a git repo. Run this from inside patrol-drone/."
  exit 1
fi

if command -v gh > /dev/null 2>&1; then
  echo "→ Using gh CLI to create and push the repo."
  echo "  This creates github.com/${ORG}/${REPO} as public and pushes main."
  gh repo create "${ORG}/${REPO}" \
    --public \
    --source=. \
    --remote=origin \
    --description "Autonomous drone patrol system: PX4 + ROS 2 Jazzy + Jetson, indoor/outdoor/forest patrol with embodied AI ambitions" \
    --push
  echo ""
  echo "✓ Done. Repo is at https://github.com/${ORG}/${REPO}"
else
  echo "gh CLI not found. Manual path:"
  echo ""
  echo "  1. Create the empty repo via the GitHub web UI:"
  echo "     https://github.com/organizations/${ORG}/repositories/new"
  echo "     - Name: ${REPO}"
  echo "     - Visibility: Public"
  echo "     - Do NOT initialize with README, .gitignore, or LICENSE (we have them)"
  echo ""
  echo "  2. Then run:"
  echo "     git remote add origin git@github.com:${ORG}/${REPO}.git"
  echo "     git push -u origin main"
  echo ""
fi
