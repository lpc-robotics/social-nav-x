# P5 preflight and DWB baseline re-freeze

The P5 preflight on 2026-09-11 found one protected-file mismatch before any P5
experiment was started.  The stable installed Nav2 template changed after the
original P0 snapshot:

| Item | SHA-256 |
|---|---|
| P0 installed `nav2.yaml` | `a42dfde54e943ac2d2cfe6b5a4d62953fcfdeb504e931abeb477ab006e82f53f` |
| Current installed `nav2.yaml` | `1b3e4986da2d049f466fc005f9bf30d55f004e90748a96b4d5ffb02a33c50dc7` |
| Current source `nav2.yaml` | `3972e35d43f3b9fda70da365b43761c310209378ea2ab57656a2cd07cd6334ef` |

The P0 installed file was upstream `HEAD` plus
`controller_frequency: 10.0`.  The current installed file additionally removes
the global costmap obstacle layer.  The source has that change and a later
10/5 Hz global update/publish-frequency edit which is not in the installed
file.  File times, the stable colcon log, and shell history show that an
independent `colcon build --packages-select arena_simulation_setup` copied the
first source state into the install at 11:31; the later source edit occurred at
11:33.  The MPC workspace did not overwrite or reset either stable file.

Because the session requires actual workspace state to take precedence, the
current installed file was accepted only after a fresh native-entry DWB smoke
on GPU 2 (UUID `GPU-2d2c45be-6725-97f3-d624-8fe61cd7ef64`, 22,057 MiB free at
selection) and private ROS domain 145.  The launch loaded
`dwb_core::DWBLocalPlanner`; navigation moved from `(3.000,3.000)` to
`(4.770,3.005)`, a 1.770 m displacement, with 83 lidar messages.  The original
P0 hash remains recorded above and in `evidence/p0`; the active protection
manifest now freezes the validated installed runtime hash.  No file in the
stable workspace was changed by this re-freeze.

Raw evidence is retained in `dwb_current_baseline_probe.log` and
`dwb_current_baseline_launch.log`.  Launch logs are intentionally ignored by
Git but remain in the local evidence directory.
