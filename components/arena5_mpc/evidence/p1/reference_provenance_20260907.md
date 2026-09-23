# P1 reference provenance and clean-room boundary

Reference checkout:

- repository: `/home/lpc/MPC-Navigation`
- commit: `5629641ffd2290d3a7ccd7eb5b7b1b3f2c0bdd64`
- reference controller SHA-256: `585b12a82983ef9c5c043327165ddab69ceb9c4620c2e83ee5d639020c2691ce`
- reference package metadata SHA-256: `acd42f332c987a718546bfe65d56a92629e06dc7111daf6f955d6e1d3af77273`

The reference `local_planner/package.xml` declares `<license>TODO</license>` and
no repository-level license file was found. The new implementation therefore
uses the mathematical behavior and independently written interfaces; it does
not copy source text from the ROS 1 Python node. Any later proposal to
redistribute copied reference code or data requires a separately established
license before P6.

The retained model facts are the unicycle state transition, reference and
control quadratic costs, terminal cost, rotated ellipse clearance, and the
discrete barrier relation. The migration deliberately corrects the old fixed
`25*j` indexing, terminal `N-1` offset, missing acceleration constraints,
non-robust angle handling, per-cycle graph construction, and reuse of shifted
commands after a failed solve.
