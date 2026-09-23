# MPC and HuNav visualization evidence

Validation date: 2026-09-15. The live test used the working tree based on
`fc16543d48a8532ea741a6e50b7d5440c5f4ee9d`; the tested implementation files
were then committed unchanged as
`c021977c52b2ac63d4b6de43a3c07facff2d4215`. The installed script and that
commit both have SHA-256
`b3ba2885603633066db82c05a26583250733a015f86760bb79b515e4c21881cf`.

The visualization node is a read-only adapter. It subscribes to `/plan`,
`/FollowPath/predicted_path`, and `/human_states`, and publishes
`/mpc/global_plan`, `/mpc/local_trajectory`, and `/mpc/human_markers`. It is not
part of `controller_server -> velocity_smoother -> watchdog -> /cmd_vel`.

## Results

- `ros_smoke.json`: isolated ROS graph PASS. It relayed three-pose global and
  local paths, produced all six human namespaces, and verified a `2.052 m`
  safety-envelope diameter for the fixture. Observed output QoS was Reliable +
  Transient Local for the global plan and Reliable + Volatile for the local
  trajectory and markers.
- `live_mpc_visualization_long_goal.json`: live Isaac/HuNav/MPC PASS on ROS
  domain 220 and GPU 3. It observed a 196-pose global plan, 28 local trajectory
  messages with a maximum of 26 poses, 286 human marker snapshots, all six
  semantic namespaces, and only `mpc_visualizer` as publisher of the three
  output topics.
- `live_navigation.json`: the bounded navigation regression completed with
  the MPC plugin, finite outputs, 377 command samples, and a conservative
  human clearance lower bound of `0.54397 m`, above the `0.30 m` gate.
- During that live run, `foxglove_bridge` listened on the intentionally chosen
  test endpoint `127.0.0.1:9015`; its process and the visualization publishers
  were present in the live graph. Production remains configurable through
  `ARENA_FOXGLOVE_ADDRESS` and `ARENA_FOXGLOVE_PORT`.

`live_mpc_visualization.json` is the first, less strict capture. It proved the
topic set but stopped after initial one-pose idle/local samples. The stricter
long-goal visualization capture supersedes it and requires a multi-pose MPC
prediction.

`live_navigation_long_goal_rejected_timeout.json` records the supplementary
8 m stochastic six-behavior run. Its observer had already passed with a
26-pose local prediction, but the unrelated navigation action exceeded the
180 s wall timeout. The probe canceled the goal and Nav2 reported `Goal
canceled`; this run is excluded from the feature gate.

The live launch shut down cleanly. A clean rebuild then passed all eight test
records: four core tests, one controller pluginlib test, one ament test record,
and the two Python visualization cases inside that ament record. Python,
launch, XML, shell, and `git diff --check` static checks also passed.

The initial visualization release was
`/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260915-c021977` and is
bound to source commit `c021977c52b2ac63d4b6de43a3c07facff2d4215`.
`release_ros_smoke.json` repeats the isolated ROS graph test using the
installed release script rather than the development overlay. The release has
zero symlinks, passed the temporary relocation and dynamic dependency audit,
and its complete SHA-256 manifest verifies. The seven protected stable files
still match. The original DWB entry was not edited.

## Path frame repair

`path_frame_pre_fix.json` is the online diagnosis against release
`20260915-c021977` on domain 222. The actual `/plan` had a `map` Path header
and 25 poses with empty `frame_id`; `/mpc/global_plan` relayed that mismatch.
At the same time the probe inspected 3,685 dynamic and nine static transforms:
none had an empty parent or child frame. The Foxglove `""` entry therefore
came from Path pose metadata, not the TF publishers.

The adapter now copies each Path and makes an empty pose frame inherit the
non-empty Path header. A genuinely conflicting non-empty pose frame is
rejected. This is applied to both global and local visualization paths and
does not modify Nav2's `/plan` or the controller's input.

`path_frame_post_fix.json` is the live development-overlay PASS on domain 225.
The raw `/plan` still had `map` plus 23 empty pose frames, while the published
`/mpc/global_plan` had `map` in its header and all 23 poses. The live TF and
TF-static samples again contained no empty transforms. `navigation_post_fix.json`
records the simultaneous successful MPC action, finite commands, and a
`0.55696 m` conservative human-clearance lower bound.

Four visualization unit cases cover inheritance, input immutability, conflict
rejection, and human markers. The complete workspace result was 10 tests,
zero errors, failures, or skips. `path_frame_ros_smoke_post_fix.json` verifies
a synthetic Nav2-style input with empty pose frames. The same strict smoke was
then repeated from immutable release `20260915-df9a55d` in
`path_frame_release_smoke_post_fix.json`; both output only `map` pose frames.
`path_frame_release_audit.json` records the release and stable-protection audit.
