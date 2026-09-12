"""Controlled RTX experiment; run with Isaac Python, exclusively in the dev tree."""
import json
from pathlib import Path

from isaacsim import SimulationApp

ROOT = Path(__file__).resolve().parents[1]
app = SimulationApp({"headless": True, "active_gpu": 2, "physics_gpu": 0,
                     "multi_gpu": False, "extra_args": [
                         f"--/log/file={ROOT}/logs/rtx_probe_kit.log",
                         "--/UJITSO/geometry=false",
                         "--/app/fastShutdown=true"]})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx")
enable_extension("omni.kit.raycast.query")
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import omni.kit.raycast.query as rq
import numpy as np
from pxr import Gf, UsdGeom
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.sensors.rtx import get_gmo_data

stage = omni.usd.get_context().get_stage()
viewport = get_active_viewport()
if viewport:
    viewport.updates_enabled = True
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
attrs = {
    "nearRangeM": 0.08, "farRangeM": 12.0, "maxReturns": 1,
    "reportRateBaseHz": 6400, "scanRateBaseHz": 10,
    "validStartAzimuthDeg": 0.0, "validEndAzimuthDeg": 360.0,
    "auxOutputType": "FULL", "skipDroppingInvalidPoints": True,
}
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar", path="/World/lidar", config=None,
    translation=Gf.Vec3d(0, 0, 1), orientation=Gf.Quatd(1, 0, 0, 0),
    force_camera_prim=False,
    **{f"omni:sensor:Core:{k}": v for k, v in attrs.items()})
for key, val in {
    "numberOfChannels": 1, "numberOfEmitters": 1, "numLines": 1,
    "numRaysPerLine": [640], "emitterState:s001:azimuthDeg": [0.0],
    "emitterState:s001:elevationDeg": [0.0],
    "emitterState:s001:fireTimeNs": [0], "emitterState:s001:channelId": [1],
}.items():
    attr = sensor.GetAttribute(f"omni:sensor:Core:{key}")
    assert attr.IsValid(), key
    attr.Set(val)

def cube(name, position, scale):
    prim = UsdGeom.Cube.Define(stage, f"/World/{name}")
    prim.CreateSizeAttr(1.0)
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    prim.AddScaleOp().Set(Gf.Vec3d(*scale))
    return prim

# Far wall, below-min-range occluder and ordinary wall, with open sectors.
cube("wall", (3, 0, 1), (0.2, 2, 2))
cube("near", (0, 0.04, 1), (0.03, 0.02, 0.1))
cube("far", (-15, 0, 1), (0.2, 5, 2))
camera = rep.create.camera(position=(0, 0, 1), look_at=(3, 0, 1),
                           clipping_range=(0.001, 100.0))
camera_rp = rep.create.render_product(camera, [320, 240])
depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
depth.attach([camera_rp.path])
rp = rep.create.render_product(sensor.GetPath(), [32, 32],
    render_vars=["GenericModelOutput", "RtxSensorMetadata"])
raw = rep.AnnotatorRegistry.get_annotator("GenericModelOutput")
raw.attach([rp.path])
flat = rep.AnnotatorRegistry.get_annotator("IsaacComputeRTXLidarFlatScan")
flat.attach([rp.path])
omni.timeline.get_timeline_interface().play()
frames = []
query = rq.acquire_raycast_query_interface()
sequence = query.add_raycast_sequence()
query_result = []
for tick in range(150):
    app.update()
    if tick == 100:
        rays = [rq.Ray((0, 0, 1), direction, 0.0, 12.0, False)
                for direction in [(1, 0, 0), (0, 1, 0), (0, -1, 0), (-1, 0, 0)]]
        print("QUERY_SUBMIT", query.submit_ray_to_raycast_sequence_array(sequence, rays), flush=True)
    if tick > 100 and not query_result:
        error, rays, results = query.get_latest_result_from_raycast_sequence_array(sequence)
        if error == rq.Result.SUCCESS:
            query_result = [{"valid": r.valid, "hit_t": r.hit_t,
                             "path": r.get_target_usd_path()} for r in results]
            print("QUERY_RESULT", query_result, flush=True)
    if tick < 90:
        continue
    data = raw.get_data()
    gmo = get_gmo_data(data)
    if gmo.magicNumber != 0x4E474D4F or gmo.numElements == 0:
        continue
    record = {"tick": tick}
    if tick == 90:
        print("GMO_MEMBERS", dir(gmo), flush=True)
    for key in ("numElements", "frameId", "timestampNs", "scanComplete",
                "elementsCoordsType", "x", "y", "z", "flags"):
        value = getattr(gmo, key)
        record[key] = np.asarray(value).tolist() if key in ("x", "y", "z", "flags") else int(value)
    for key in ("objId", "matId", "hitNormals", "emitterId", "channelId"):
        try:
            record[key] = np.asarray(getattr(gmo, key)).tolist()
        except Exception:
            pass
    scan = flat.get_data()
    record["flat"] = {k: np.asarray(v).tolist() for k, v in scan.items() if k != "info"}
    frames.append(record)
out = ROOT / "audit/rtx_rays.json"
out.write_text(json.dumps(frames, indent=2))
(ROOT / "audit/rtx_query.json").write_text(json.dumps(query_result, indent=2))
query.remove_raycast_sequence(sequence)
np.save(ROOT / "audit/probe_camera_depth.npy", depth.get_data())
print(f"PROBE_RESULT {out} frames={len(frames)}", flush=True)
app.close()
