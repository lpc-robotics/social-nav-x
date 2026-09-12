# arena5 MPC development workspace

This repository contains the isolated development, evidence, and eventual additive release artifacts for migrating the MPC core from `/home/lpc/MPC-Navigation` to `/home/lpc/workspace/arena5_ws`.

The stable workspace is a read-only underlay during P0-P5. Existing files in it must not be edited, rebuilt, reset, or cleaned. The original DWB entry remains the default.

Current phase: P0 through P6 passed.  The immutable release is installed at
`/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260912-58cd661`, and the
only new stable-workspace entry point is
`/home/lpc/workspace/arena5_ws/scripts/run_six_behaviors_mpc.sh`.  The original
`run_six_behaviors.sh` remains the default DWB entry and retains its protected
hash.

Phase evidence is under `evidence/p0` through `evidence/p6`.  P1 contains the
locked CasADi SDK evidence, independently implemented `arena_mpc_core`,
Python/C++ numerical comparisons, and capacity benchmarks.  P2-P4 cover Nav2
integration, fault handling, static navigation, and dynamic-human safety.  P5
contains the maximum-load benchmark, paired DWB/MPC comparison, and accepted
30-minute endurance run.  P6 records the relocatable release audit and final
MPC/DWB rollback smoke.

Run the released controller from the stable workspace with:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors_mpc.sh
```

Run the unchanged DWB default with:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

The governing plan is `/home/lpc/workspace/ARENA5_MPC_MIGRATION_PLAN_REV2.md`.
