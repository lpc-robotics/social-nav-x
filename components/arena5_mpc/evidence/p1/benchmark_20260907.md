# P1 obstacle-layout benchmark

Date: 2026-09-07 (Asia/Shanghai)

Each row below contains 1000 repeated solves after one cold solve. Times are
wall-clock milliseconds. `maximum` means that the core received and
postchecked 32 dynamic plus 128 static obstacle predictions; the conservative
reachable set contained the configured maximum of eight dynamic NLP objects.
All static objects remain part of the independent postcheck.

```csv
layout,load,condition,iterations,graph_build_ms,cold_parameter_ms,cold_solve_ms,cold_postcheck_ms,warm_p50_ms,warm_p95_ms,warm_p99_ms,warm_max_ms,success,timeout,dynamics_residual,geometry_violation,barrier_violation,cold_code,cold_status
exact_active,none,feasible,1000,365.345765,0.025179,6.882953,0.003422,3.312742,4.442303,4.527924,6.545337,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
exact_active,six_dynamic,feasible,1000,287.352626,0.015123,21.308888,0.008616,17.960940,18.999219,19.361461,19.604830,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
exact_active,six_dynamic,critical,1000,274.913148,0.013112,21.192604,0.009797,19.820679,20.885187,21.164906,21.864341,1000,0,0.000001,0.000000,0.000000,0,Solve_Succeeded
exact_active,six_dynamic,infeasible,1000,277.503727,0.013154,63.019999,0.000000,62.834143,64.343142,65.337175,66.382594,0,1000,0.000000,0.000000,0.000000,4,Maximum_CpuTime_Exceeded
exact_active,maximum,feasible,1000,346.361557,0.013843,24.884525,0.169456,24.509742,25.940587,26.468317,31.558628,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
exact_active,maximum,critical,1000,359.369074,0.013653,25.315585,0.170681,24.631798,25.885243,26.371722,30.480209,1000,0,0.000002,0.000000,0.000000,0,Solve_Succeeded
exact_active,maximum,infeasible,1000,366.149581,0.021180,64.327051,0.000000,63.376502,65.113241,65.864248,67.468187,0,1000,0.000000,0.000000,0.000000,4,Maximum_CpuTime_Exceeded
fixed_masked,none,feasible,1000,354.543423,0.012096,17.144113,0.002330,13.898015,14.649289,15.120341,16.629835,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
fixed_masked,six_dynamic,feasible,1000,346.237118,0.003831,22.524788,0.009300,21.783926,23.130920,23.735169,29.400255,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
fixed_masked,six_dynamic,critical,1000,356.042595,0.004001,26.459133,0.009689,24.743274,26.032452,26.561257,30.661595,1000,0,0.000001,0.000000,0.000000,0,Solve_Succeeded
fixed_masked,six_dynamic,infeasible,1000,356.361618,0.004420,62.792287,0.000000,63.384502,65.099548,65.967491,68.221121,0,1000,0.000000,0.000000,0.000000,4,Maximum_CpuTime_Exceeded
fixed_masked,maximum,feasible,1000,344.737736,0.004776,25.190381,0.148805,24.627714,25.885494,26.394407,27.018189,1000,0,0.000000,0.000000,0.000000,0,Solve_Succeeded
fixed_masked,maximum,critical,1000,352.999314,0.004672,25.293373,0.168764,24.817662,25.903605,26.458369,31.097724,1000,0,0.000002,0.000000,0.000000,0,Solve_Succeeded
fixed_masked,maximum,infeasible,1000,353.927708,0.004231,64.328674,0.000000,63.365178,65.188212,66.031344,67.866105,0,1000,0.000000,0.000000,0.000000,4,Maximum_CpuTime_Exceeded
```

All feasible and critical rows completed 1000/1000 solves. The chosen fixed
layout has worst feasible p95/p99 of 26.03/26.56 ms, parameter packing below
0.005 ms in the cold samples, and full-input postcheck below 0.17 ms. Its graph
build takes about 0.35 s and is performed during solver construction in the
plugin configuration phase.

The deliberately infeasible initial-overlap rows exhausted IPOPT's 60 ms CPU
limit and returned after wall p95 65.19 ms at worst. All 6000 infeasible solves
returned invalid commands. IPOPT's CPU limit is not a hard wall deadline; the
90 ms plugin commit rule and independent watchdog remain necessary in P2.

## Rejected layouts

The initial literal fixed-size formulation put 32 dynamic and 128 static
objects, 4000 explicit slack variables, and all state variables in one NLP. A
pilot measured graph construction at about 8 seconds and solve time around
98--151 ms even with no or six active objects. Analytically eliminating slack
did not make a 160-slot NLP viable: graph construction reached 28 seconds and
solve times remained around 290--407 ms in a single-shooting pilot.

The data reject a literal 160-slot fixed NLP. The selected design keeps the
32/128 input capacities, proves which dynamic objects cannot enter the robot's
maximum reachable disc over the horizon, stops if more than eight dynamic
objects remain relevant, keeps static geometry in costmap collision checking,
and postchecks every received object. A fixed eight-slot graph is preferable to
exact-active graphs because its small timing cost avoids 0.27--0.37 s graph
creation whenever the active count changes.
