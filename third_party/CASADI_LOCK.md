# CasADi dependency lock

Candidate validated in P1: CasADi 3.8.0 official PyPI wheel for CPython stable ABI on manylinux 2.28 x86_64.

```text
filename: casadi-3.8.0-cp311-abi3-manylinux_2_28_x86_64.whl
sha256: b58ae6f3784b5553461d70de9848ca5c893e5ae3eb81d00959dce2ce8484da58
source: https://files.pythonhosted.org/ via pip index
official release page: https://web.casadi.org/get/
```

The artifact is unpacked only under `third_party/casadi-3.8.0-wheel` during development. It contains the C++ headers, CMake package, `libcasadi.so.3.7`, the CasADi IPOPT adapter, IPOPT 3.14.19, MUMPS/METIS, OpenBLAS and private Fortran runtimes.

This lock does not authorize installation into the stable ROS or Isaac environments. The eventual release must copy only the runtime closure required by the new MPC packages and retain `$ORIGIN`-relative lookup.

The wheel metadata declares `LGPLv3+` for CasADi and ships third-party license
texts under `casadi/include/licenses`. P6 must carry the applicable CasADi,
IPOPT, MUMPS/METIS, BLAS, Fortran-runtime, zlib, and other closure notices with
the release artifact; copying shared libraries without their notices is not an
acceptable release package.
