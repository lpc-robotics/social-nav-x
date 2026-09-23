# P1 CasADi C++ SDK and IPOPT evidence

Date: 2026-09-07 (Asia/Shanghai)

This check used only the isolated workspace. Nothing was installed into the
stable Arena workspace, its RoboStack environment, or the Isaac environment.

## Candidate artifact

- CasADi: 3.8.0 official PyPI `cp311-abi3-manylinux_2_28_x86_64` wheel
- SHA-256: `b58ae6f3784b5553461d70de9848ca5c893e5ae3eb81d00959dce2ce8484da58`
- Unpacked development prefix: `third_party/casadi-3.8.0-wheel`
- Target compiler: `/home/lpc/workspace/arena5_ws/.conda/arena_ros/bin/x86_64-conda-linux-gnu-c++`
- Compiler detected by CMake: GCC 15.3

The artifact contains `casadi/casadi.hpp`, a CMake config exporting
`casadi::casadi`, `libcasadi.so.3.7`, `libcasadi_nlpsol_ipopt.so.3.7`, IPOPT
3.14.19, MUMPS/METIS, the CasADi OpenBLAS library, and private Fortran runtime
libraries.

## Native C++ result

`p1/casadi_cpp_smoke` was configured with the target RoboStack compiler, built
as C++17 with warnings as errors, and linked through the exported CMake target.
It queried `has_nlpsol("ipopt")`, created an IPOPT NLP, and solved it:

```text
CASADI_CPP_SMOKE_OK version=3.8.0 solution=3 status=Solve_Succeeded
```

## Relocation and runtime closure

A staging tree containing the executable and only its transitive CasADi/IPOPT
runtime closure was run with an empty environment except for `/usr/bin:/bin`.
No `CASADI_PLUGIN_PATH` or development prefix was present, and the solve still
succeeded. The CasADi core has `$ORIGIN` RUNPATH and the IPOPT adapter has
`$ORIGIN:$ORIGIN/.` RPATH, so sibling plugin discovery works.

The staged IPOPT adapter resolves the expected closure: CasADi, IPOPT,
coinmumps, coinmetis, the private gfortran and quadmath libraries, CasADi
OpenBLAS, zlib, libmvec, and system glibc/C++ libraries. There were no unresolved
entries in `ldd`.

The smoke executable's install RPATH also contains the frozen RoboStack library
directory because the selected compiler toolchain injected it. This is valid
for P1 development but is not the final release contract. P6 must stage and
test the complete overlay while hiding all development paths.

## Gate conclusion

The standalone C++ SDK, CMake package, compiler ABI, IPOPT plugin, runtime
closure, and sibling-library relocation checks pass. Actual pluginlib loading,
Nav2 controller lifecycle, and in-process solving remain P2 tests and must not
be described as verified yet.
