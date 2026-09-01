# Character frame correction retest

Date: 2026-09-01. Worktree: `feature/formal-social-automata-v1`.

- Root cause: ROS planar poses use local `+X` as forward; Isaac People assets
  render local `-Y` as forward.
- Boundary conversion: `q_character = q_ros * qz(+pi/2)`; graph feedback uses
  the inverse `q_ros = q_character * qz(-pi/2)`.
- Tests: character conversion `17/17`, bridge `20/20`, formal `76/76`; scoped
  colcon results are `20/0/0/0` and `261/0/0/0` test/subtest.
- Sudden visual log: `a10f4a3a60f58a438b75416cc455e40ed6e91cc63bf57ee738d9842585b6a694`.
- Safe visual log: `45f211e5ffa60707d7994c0adc8e6cad568542025e6be6017becf4215738e13f`.
- Fast visual log: `2bd9cb5bc96c1df699e22370eb719ee5f392fe07eaa3a93ff86dd38d19ed13a1`.
- Strict six-behavior marker log:
  `25cc7d45680f71068512a8fcc9223ba3feb9bea88e4ea34cafc2a0af6fe3e811`.
- Patch: `1d404fca247c81a6dfbfaa470cfd35a7ca8005d0b2cd2be056fd9bf141875b23`,
  31 files, forward/reverse checks and applied-tree comparisons passed.
- `activity_before.sha256` and `activity_after.sha256` contain identical
  protected hashes. No activity workspace dependency or source was changed.
