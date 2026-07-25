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

## Flipping the lane onto the image

Once a digest is published (first build runs when this lands on `main`, or on demand via
dispatch), a small reviewable PR flips `.github/workflows/replay-regression.yml`:

1. Grant the workflow `packages: read` so `GITHUB_TOKEN` can pull the (private, repo-linked) image
   — **required**: without it the token pulls anonymously and GHCR returns `manifest unknown`, not
   a clear auth error. Add it to the top-level `permissions:` block:

   ```yaml
   permissions:
     contents: read
     packages: read
   ```

2. Point the job container at the image and add pull credentials (with the scope above,
   `GITHUB_TOKEN` can read the repo-linked package):

   ```yaml
   container:
     image: ghcr.io/sentientswarm/patrol-drone/replay-lane@sha256:<digest from the job summary>
     credentials:
       username: ${{ github.actor }}
       password: ${{ secrets.github_token }}
   ```

3. Delete the two job-time apt steps (**Install git-lfs** and **Install the ROS runtime …**) —
   everything they install is baked in. The env probe, overlay cache/build, and test steps are
   unchanged (`ROS_DISTRO` is baked into the image but the job env keeps it manifest-synced).

## Updating the image

Rebuilding (a Dockerfile edit, or a re-run to pick up upstream security patches) produces a new
digest; bumping the digest in `replay-regression.yml` is a normal reviewable PR — the same flow the
repo already uses for its digest-pinned base images and actions. The apt manifest inside the image
diffs the exact package-version changes between two digests.
