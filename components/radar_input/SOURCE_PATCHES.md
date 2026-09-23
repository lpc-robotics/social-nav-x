# Radar input source patch manifest

The accepted radar-input implementation was developed in isolated Git
worktrees. The root component history preserves its contracts, validation,
tools, and recovery manifests; this directory additionally stores the actual
incremental source diffs so a fresh `social-nav-x` checkout is self-contained.

The normalized LaserScan path remains opt-in. Do not merge its
`simulation-setup` patch into the stable default configuration.

## Arena Isaac

Upstream base: `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c`

The repository-level `patches/arena-isaac.patch` reconstructs the protected
Arena Isaac baseline corresponding to local snapshot `c5a1d8e`. Apply these
incremental patches in order when enabling radar input:

1. `arena-isaac-depth-clearing.patch`: `c5a1d8e..be8fefc`
2. `arena-isaac-normalized-laserscan.patch`: `be8fefc..6709da1`

The accepted Arena Isaac tip is `6709da1`, tagged `accepted/radar-input-v1` in
the source worktree.

## Simulation setup

Upstream base: `3f142b25d88ce962c803b57cf20f38985d376dea`

The repository-level `patches/simulation-setup.patch` reconstructs the active
stable baseline snapshot `06574f3`. Apply these incremental patches in order:

1. `simulation-setup-depth-clearing.patch`: `06574f3..388e546`
2. `simulation-setup-normalized-laserscan.patch`: `388e546..766c435`

The second patch selects `/lidar_normalized` as the Nav2 observation source and
must only be used by the isolated normalized-radar launch. The accepted
simulation-setup tip is `766c435`, also tagged `accepted/radar-input-v1` in its
source worktree.

## SHA-256

```text
2db99aa40c02f312f8abe57f758e992b0001204f156f15150b8f0277414461de  arena-isaac-depth-clearing.patch
e361f3317891b41cfbf2d61c7fd6bf0f83a7c45f3ff4aa53fc62274e5de396f3  arena-isaac-normalized-laserscan.patch
b2e3af244208297c0f36408beb18bd6e026bee9b31c67fb52eba51cb03033e58  simulation-setup-depth-clearing.patch
85a02ef648ba09ab615f125272570d24c0c72894b2b52924446d41b20d62fb09  simulation-setup-normalized-laserscan.patch
```

Apply patches with `git apply --check` first and only against the exact commits
listed above. Runtime selection, validation boundaries, and rollback steps are
documented in `RADAR_INPUT_HANDOFF.md`.
