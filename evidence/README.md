# Validation evidence

This directory contains the compact, authoritative artifacts selected from the
final D6 backup:

- `baseline/results.csv`: pre-D6 comparison data;
- `matrix/`: 22 isolated fresh-world velocity and hold-out cases;
- `retest/`: standalone `(0.7,-0.6)` re-test;
- `collision/`: frontal, oblique, and combined-turn wall tests;
- `system/`: final Nav2/HuNav/visualization/sensor summary and WebRTC frame;
- `completion_audit.txt`: machine-checked completion gates.

Large Isaac and ROS runtime logs are deliberately excluded from Git. Their
original workspace paths remain recorded in `CHASSIS_CONTROL.md`.
