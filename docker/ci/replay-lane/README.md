# replay-lane CI image

The pre-built, digest-pinned container image for the `replay-regression` CI lane
(PR #16, Mira Medium F-06 / review 4728294643: *"an immutable/signed apt snapshot or a reviewed
image with the required tools"* — this is the reviewed image).

The lane previously apt-installed its runtime (git-lfs, the ROS runtime/type-support set, rsync)
at job time from mutable package indexes. This image bakes that exact runtime once at image-build
time and freezes the resolved set by digest; the lane then pulls
`ghcr.io/sentientswarm/patrol-drone/replay-lane@sha256:<digest>` and runs zero job-time installs.
An auditable manifest of the exact frozen versions is baked at
`/usr/local/share/patrol-drone/replay-lane-apt.manifest`.

## Build + publish

`.github/workflows/build-replay-lane-image.yml` builds this Dockerfile and pushes it to GHCR
(`packages: write` via `GITHUB_TOKEN`, which auto-links the package to this repo). It runs:

- on `workflow_dispatch` (Actions → *Build replay-lane image* → Run workflow), and
- on pushes to `main` that touch `docker/ci/replay-lane/**` (so a Dockerfile change republishes).

The job summary prints the published **digest** — that digest, not a tag, is what the lane pins.

## How the lane was flipped onto the image (done — SWM-84, PR #20)

`.github/workflows/replay-regression.yml` now pulls this image and runs **zero** job-time apt
installs. What the flip consisted of, kept as the rationale for the workflow's current shape — and
as the recipe if another lane is ever moved onto a pinned image:

1. **`packages: read` on the workflow** so `GITHUB_TOKEN` can pull the (private, repo-linked) image
   — **required**: without it the token pulls anonymously and GHCR returns `manifest unknown`, not
   a clear auth error. It lives in the top-level `permissions:` block:

   ```yaml
   permissions:
     contents: read
     packages: read
   ```

2. **The job container points at the digest**, with pull credentials (with the scope above,
   `GITHUB_TOKEN` reads the repo-linked package):

   ```yaml
   container:
     image: ghcr.io/sentientswarm/patrol-drone/replay-lane@sha256:<digest from the job summary>
     credentials:
       username: ${{ github.actor }}
       password: ${{ secrets.github_token }}
   ```

3. **The two job-time apt steps were deleted** (*Install git-lfs* and *Install the ROS runtime …*)
   — everything they installed is baked in. The env probe, overlay cache/build, and test steps were
   unchanged (`ROS_DISTRO` is baked into the image, but the job env keeps it manifest-synced). The
   workflow file is also folded into the overlay cache key, so a digest repin self-invalidates a
   stale compiled overlay.

## Updating the image

Rebuilding (a Dockerfile edit, or a re-run to pick up upstream security patches) produces a new
digest; bumping the digest in `replay-regression.yml` is a normal reviewable PR — the same flow the
repo already uses for its digest-pinned base images and actions. The apt manifest inside the image
diffs the exact package-version changes between two digests.
