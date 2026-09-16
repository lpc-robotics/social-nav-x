# Isaac depth-clearing runtime overlay

This runtime tree is a byte-for-byte copy of the existing Arena Isaac Python
packages plus the validated depth-clearing change from the isolated
`arena5_ws_costmap_fix` workspace. The functional change is bound to upstream
commit `be8fefce4fdeb238372da923214e478d92bbbc32`; its unchanged Arena baseline was
captured by `c5a1d8e9561b869c7c67898007948460e6d892ec`.

The overlay leaves `/lidar` unchanged. Four co-located render-depth views
publish `/lidar_clearing` endpoints. The MPC Nav2 configuration consumes that
PointCloud2 source with `marking=false` and `clearing=true` only. Invalid depth
values cannot produce clearing points, and finite endpoints stop 0.05 m before
the rendered surface.

The immutable release runner prepends this directory to `PYTHONPATH` and
defaults `ARENA_DEPTH_CLEARING=true`. Setting `ARENA_DEPTH_CLEARING=false` at
the next launch disables the extra render products and publisher without
modifying any source or release file.
