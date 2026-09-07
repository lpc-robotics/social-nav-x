# arena5 MPC development workspace

This repository contains the isolated development, evidence, and eventual additive release artifacts for migrating the MPC core from `/home/lpc/MPC-Navigation` to `/home/lpc/workspace/arena5_ws`.

The stable workspace is a read-only underlay during P0-P5. Existing files in it must not be edited, rebuilt, reset, or cleaned. The original DWB entry remains the default.

Current phase: P1 passed; P2 interface, timing, and fault integration is next.

P0 evidence is under `evidence/p0`. P1 contains the locked CasADi SDK evidence,
the independently implemented `arena_mpc_core` package, Python/C++ numerical
comparisons, and the 1000-run-per-case capacity benchmark under `evidence/p1`.

The governing plan is `/home/lpc/workspace/ARENA5_MPC_MIGRATION_PLAN_REV2.md`.
