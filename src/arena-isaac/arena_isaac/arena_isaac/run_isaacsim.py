# fmt: off


# preload attrs
import os
import sys
import arena_simulation_setup
try:
    import arena_simulation_setup.utils.cattrs
except ModuleNotFoundError:
    # arena-rosnav/humble pins simulation-setup before this helper existed.
    pass

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

def _arg_value(name: str, default):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    prefix = f"{name.lstrip('-')}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default

def _arg_bool(name: str, default: bool) -> bool:
    return str(_arg_value(name, default)).lower() in ("true", "1")

LIVESTREAM = _arg_bool("--livestream", False)
INTERNAL_GPU = int(os.environ.get("ARENA_INTERNAL_GPU", "0"))
RENDER_GPU = int(os.environ.get("ARENA_RENDER_GPU", str(INTERNAL_GPU)))
IDEAL_CHASSIS = str(os.environ.get("ARENA_IDEAL_CHASSIS", "false")).lower() in ("true", "1")
PHYSICS_DT = float(os.environ.get("ARENA_PHYSICS_DT", str(1.0 / 60.0)))
EXTRA_ARGS = []
if kit_log := os.environ.get("ARENA_KIT_LOG"):
    EXTRA_ARGS.append(f"--/log/file={kit_log}")
if LIVESTREAM:
    webrtc_ip = _arg_value("--webrtc-ip", os.environ.get("ARENA_WEBRTC_IP", "127.0.0.1"))
    signal_port = int(_arg_value("--webrtc-signal-port", "49100"))
    media_port = int(_arg_value("--webrtc-media-port", "47998"))
    EXTRA_ARGS += [
        f"--/app/livestream/publicEndpointAddress={webrtc_ip}",
        f"--/app/livestream/port={signal_port}",
        f"--/app/livestream/fixedHostPort={media_port}",
        "--enable",
        "omni.services.livestream.nvcf",
    ]

CONFIG = {
    "renderer": "Wireframe",
    "headless": _arg_bool("--headless", False) or LIVESTREAM,
    "hide_ui": False if LIVESTREAM else None,
    # Kit/Vulkan keeps the host GPU ordinal while CUDA uses the remapped
    # ordinal after CUDA_VISIBLE_DEVICES (normally cuda:0).
    "active_gpu": RENDER_GPU,
    "physics_gpu": INTERNAL_GPU,
    "multi_gpu": False,
    "max_gpu_count": 1,
    "extra_args": EXTRA_ARGS,
}
#import parent directory
from pathlib import Path

simulation_app = SimulationApp(CONFIG)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(parent_dir))

# stdlib
import queue
import random
import time
import traceback

# Import Isaac Sim dependencies

import carb
import omni.kit.commands as commands
import omni.timeline
import omni.usd
import yaml
from isaac_utils.utils.assets import get_assets_root_path_safe
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.asset.importer.urdf import _urdf
from omni.isaac.core import SimulationContext, World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils import extensions, prims, stage
from pxr import Sdf

EXTENSIONS_PEOPLE = [
    'omni.anim.people',
    'omni.anim.navigation.bundle',
    'omni.anim.timeline',
    'omni.anim.graph.bundle',
    'omni.anim.graph.core',
    'omni.anim.retarget.bundle',
    'omni.anim.retarget.core',
    'omni.kit.scripting',
    'omni.graph.nodes',
    'omni.anim.curve.core',
    'omni.anim.navigation.core',
]

EXTENSIONS_MATERIAL = [
    'omni.kit.material.library',
]

if not CONFIG["headless"]:
    EXTENSIONS_PEOPLE += ['omni.anim.graph.ui', 'omni.anim.retarget.ui']
    EXTENSIONS_MATERIAL += [
        'omni.kit.browser.material',
        'omni.kit.browser.asset',
        'omni.kit.window.material',
    ]

for ext_people in EXTENSIONS_PEOPLE:
    extensions.enable_extension(ext_people)

for ext_material in EXTENSIONS_MATERIAL:
    extensions.enable_extension(ext_material)

import tomllib

def enable_extensions_from_kit(kit_path):
    with open(kit_path, "rb") as f:
        data = tomllib.load(f)
        dependencies = data.get("dependencies", {})

        for ext_name in dependencies.keys():
            print(f"Enabling: {ext_name}")
            enable_extension(ext_name)

KIT_FILE_PATH = (
    os.path.join(os.environ["EXP_PATH"], "isaacsim.exp.base.kit")
    if CONFIG["headless"]
    else os.path.join(os.environ["EXP_PATH"], "isaacsim.exp.full.kit")
)
enable_extensions_from_kit(KIT_FILE_PATH)

# Update the simulation app with the new extensions
for _ in range(100):
    simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
omni.usd.get_context().new_stage()

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.sensors.physics")
extensions.enable_extension("isaacsim.sensors.camera")

import numpy as np

#Import world generation dependencies
import omni.anim.graph.core as ag

#imprt navmesh gen
import omni.anim.navigation.core as nav
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

# rclpy
import rclpy
import rclpy.node
import std_srvs.srv
from geometry_msgs.msg import Twist

# graphs
from isaac_utils.graphs.time import PublishTime
from isaac_utils.managers import entity_lifecycle
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.path import world_path

#Import services
from arena_isaac.services import services
from pedestrian.simulator.logic.people_manager import PeopleManager
from rclpy.qos import QoSProfile
from arena_isaac import run_after_tick_queue

# fmt: on
# ======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
# BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
plane_material_paths = [
    'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/Base/Wood/Walnut_Planks.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl'
]
world = (
    World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
    if IDEAL_CHASSIS
    else World()
)
world.scene.add_ground_plane(size=100, z_position=0.0)
Material.physics(
    parent_prim_path=world_path(),
    key='ground_default',
    params=PhysicsParams(
        static_friction=1.0,
        dynamic_friction=1.0,
        restitution=0.0,
        combine_mode='min',
    ),
).bind_to('/World/groundPlane/collisionPlane')
_stage = omni.usd.get_context().get_stage()
plane_mdl_path = random.choice(plane_material_paths)
plane_mtl_name = plane_mdl_path.split('/')[-1][:-4]
plane_mtl_path = "/World/Looks/PlaneMaterial"
plane_mtl = _stage.GetPrimAtPath(plane_mtl_path)
# if not (plane_mtl and plane_mtl.IsValid()):
#     create_res = omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
#                                                 mtl_url=plane_mdl_path,
#                                                 mtl_name=plane_mtl_name,
#                                                 mtl_path=plane_mtl_path)

#     bind_res = omni.kit.commands.execute('BindMaterialCommand',
#                                             prim_path="/World/groundPlane",
#                                             material_path=plane_mtl_path)
simulation_app.update()  # update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0)  # currently we use 1m for simulation.
light_1 = prims.create_prim(
    "/World/Light_1",
    "DomeLight",
    position=np.array([1.0, 1.0, 1.0]),
    attributes={
        "inputs:texture:format": "latlong",
        "inputs:intensity": 1000.0,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
assets_root_path = get_assets_root_path_safe()

# Navmesh config and baking
simulation_app.update()
stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute("CreateNavMeshVolumeCommand",
                          parent_prim_path=Sdf.Path("/World"),
                          layer=stage.GetRootLayer()
                          )
simulation_app.update()

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
    value=35.0)

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
    value=True)
omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/navmesh_enabled',
    value=True)

inav = nav.acquire_interface()
x = inav.start_navmesh_baking()
simulation_app.update()


# =================================================================================

# ===================================controller====================================
# create controller node for isaacsim.


class IsaacController(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(node_name="isaac", *args, **kwargs)
        self._running = True
        self._should_step_once = False
        self._ideal_command_v = 0.0
        self._ideal_command_w = 0.0
        self._ideal_reference_v = 0.0
        self._ideal_reference_w = 0.0
        self._ideal_command_time = 0.0
        self._ideal_articulation_path = ""
        self._ideal_articulation = None

        self._ideal_max_linear = float(
            os.environ.get("ARENA_IDEAL_MAX_LINEAR", "1.5")
        )
        self._ideal_max_angular = float(
            os.environ.get("ARENA_IDEAL_MAX_ANGULAR", "2.0")
        )
        self._ideal_linear_acceleration = float(
            os.environ.get("ARENA_IDEAL_LINEAR_ACCELERATION", "2.0")
        )
        self._ideal_angular_acceleration = float(
            os.environ.get("ARENA_IDEAL_ANGULAR_ACCELERATION", "4.0")
        )
        self._ideal_command_timeout = float(
            os.environ.get("ARENA_IDEAL_COMMAND_TIMEOUT", "0.5")
        )

        if IDEAL_CHASSIS:
            self._ideal_cmd_subscription = self.create_subscription(
                Twist,
                "/cmd_vel",
                self._cb_ideal_cmd_vel,
                10,
            )
            self.get_logger().info(
                "Ideal planar chassis enabled: physics_dt=%.6f, "
                "limits=(%.3f m/s, %.3f rad/s), accelerations=(%.3f m/s^2, %.3f rad/s^2)"
                % (
                    PHYSICS_DT,
                    self._ideal_max_linear,
                    self._ideal_max_angular,
                    self._ideal_linear_acceleration,
                    self._ideal_angular_acceleration,
                )
            )

        self.__pause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/PauseSimulation'),
            self._cb_pause,
        )
        self.__unpause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/UnpauseSimulation'),
            self._cb_unpause,
        )
        self.__step_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/StepSimulation'),
            self._cb_step,
        )

    def _cb_pause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = False
        response.success = True
        return response

    def _cb_unpause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = True
        response.success = True
        return response

    def _cb_step(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._should_step_once = True
        response.success = True
        return response

    @property
    def _step_once(self) -> bool:
        v = self._should_step_once
        self._should_step_once = False
        return v

    @property
    def running(self):
        return self._step_once or self._running

    @staticmethod
    def _approach(current: float, target: float, maximum_step: float) -> float:
        difference = max(-maximum_step, min(maximum_step, target - current))
        return current + difference

    def _cb_ideal_cmd_vel(self, command: Twist):
        self._ideal_command_v = max(
            -self._ideal_max_linear,
            min(self._ideal_max_linear, float(command.linear.x)),
        )
        self._ideal_command_w = max(
            -self._ideal_max_angular,
            min(self._ideal_max_angular, float(command.angular.z)),
        )
        self._ideal_command_time = time.monotonic()

    def _resolve_ideal_articulation(self):
        manifests = entity_lifecycle.manifests_snapshot()
        if not manifests:
            self._ideal_articulation = None
            self._ideal_articulation_path = ""
            return None

        articulation_path = next(iter(manifests.values())).articulation_path
        if (
            self._ideal_articulation is not None
            and articulation_path == self._ideal_articulation_path
        ):
            return self._ideal_articulation

        try:
            articulation = SingleArticulation(
                prim_path=articulation_path,
                name="arena_ideal_chassis",
            )
            articulation.initialize()
        except Exception as error:
            carb.log_warn(
                f"Ideal chassis waiting for articulation {articulation_path}: {error}"
            )
            return None

        self._ideal_articulation = articulation
        self._ideal_articulation_path = articulation_path
        self._ideal_reference_v = 0.0
        self._ideal_reference_w = 0.0
        carb.log_info(f"Ideal planar chassis attached to {articulation_path}")
        return articulation

    def apply_ideal_chassis(self, dt: float):
        """Set planar root twist before PhysX resolves the next step.

        This models an ideal velocity-controlled mobile base without writing
        pose. A D6 joint authored at spawn owns Z/roll/pitch, while the normal
        PhysX solver remains responsible for chassis contact and collision.
        """
        if not IDEAL_CHASSIS:
            return

        articulation = self._resolve_ideal_articulation()
        if articulation is None:
            return

        command_fresh = (
            time.monotonic() - self._ideal_command_time
            <= self._ideal_command_timeout
        )
        target_v = self._ideal_command_v if command_fresh else 0.0
        target_w = self._ideal_command_w if command_fresh else 0.0
        self._ideal_reference_v = self._approach(
            self._ideal_reference_v,
            target_v,
            self._ideal_linear_acceleration * dt,
        )
        self._ideal_reference_w = self._approach(
            self._ideal_reference_w,
            target_w,
            self._ideal_angular_acceleration * dt,
        )

        try:
            _, orientation = articulation.get_world_pose()
            orientation = np.asarray(orientation, dtype=float)
            quat_w, quat_x, quat_y, quat_z = orientation
            yaw = np.arctan2(
                2.0 * (quat_w * quat_z + quat_x * quat_y),
                1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z),
            )
            current_linear = np.asarray(
                articulation.get_linear_velocity(), dtype=float
            )
            current_angular = np.asarray(
                articulation.get_angular_velocity(), dtype=float
            )
            articulation.set_linear_velocity(
                np.array(
                    [
                        np.cos(yaw) * self._ideal_reference_v,
                        np.sin(yaw) * self._ideal_reference_v,
                        current_linear[2],
                    ],
                    dtype=np.float32,
                )
            )
            articulation.set_angular_velocity(
                np.array(
                    [
                        current_angular[0],
                        current_angular[1],
                        self._ideal_reference_w,
                    ],
                    dtype=np.float32,
                )
            )
        except Exception as error:
            carb.log_warn(f"Ideal chassis articulation view invalidated: {error}")
            self._ideal_articulation = None
            self._ideal_articulation_path = ""

    @classmethod
    def wait_for_bridge(cls):
        extensions.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        from isaacsim.ros2.bridge._ros2_bridge import acquire_ros2_bridge_interface
        ros2_bridge = acquire_ros2_bridge_interface()
        while not ros2_bridge.get_startup_status():
            simulation_app.update()

        carb.log_info("ROS 2 bridge started successfully!")


# ======================================main=======================================


def main(args=None):
    """
    Main function to initialize the simulation, create the ROS 2 node,
    and run the simulation loop.
    """

    sim = SimulationContext()

    IsaacController.wait_for_bridge()
    rclpy.init(args=[])
    controller = IsaacController()

    for service in services:
        service.create(controller, qos_profile=QoSProfile(depth=2000))

    PublishTime('/World/publish_time')
    world.reset()

    screenshot_path = os.environ.get("ARENA_SCREENSHOT", "")
    screenshot_after = float(os.environ.get("ARENA_SCREENSHOT_DELAY", "45"))
    screenshot_started_at = time.monotonic()
    screenshot_requested = False
    stream_viewport = None
    if LIVESTREAM or screenshot_path:
        try:
            from omni.kit.viewport.utility import get_active_viewport

            camera_path = "/World/ArenaOverviewCamera"
            overview_camera = prims.create_prim(
                camera_path,
                "Camera",
                position=np.array([15.0, 11.5, 28.0]),
            )
            overview_camera.GetAttribute("focalLength").Set(18.0)
            stream_viewport = get_active_viewport()
            if stream_viewport:
                stream_viewport.set_active_camera(camera_path)
                carb.log_info(f"Arena livestream camera ready: {camera_path}")
        except Exception as exc:
            carb.log_warn(f"Could not configure Arena livestream camera: {exc}")

    # set photoreal settings
    import isaac_utils.config.photoreal as photoreal
    if os.environ.get('RENDER_PRESET', 'photoreal') != 'boring':
        photoreal.PRESET_PHOTOREAL.apply()
    else:
        photoreal.PRESET_DEFAULT.apply()

    # hard reset once
    omni.timeline.get_timeline_interface().stop()

    # mainloop
    was_playing: bool = False
    try:
        while simulation_app.is_running():
            stepped_this_iteration: bool = False
            rclpy.spin_once(controller, timeout_sec=0)
            if controller.running:
                if not was_playing:
                    world.play()
                    was_playing = True
                controller.apply_ideal_chassis(PHYSICS_DT)
                world.step(render=True)
                stepped_this_iteration = True
            else:
                if was_playing:
                    world.stop()
                    was_playing = False
                simulation_app.update()

            if stepped_this_iteration:
                if (
                    screenshot_path
                    and not screenshot_requested
                    and stream_viewport
                    and time.monotonic() - screenshot_started_at >= screenshot_after
                ):
                    from omni.kit.viewport.utility import capture_viewport_to_file

                    capture_viewport_to_file(stream_viewport, file_path=screenshot_path)
                    screenshot_requested = True
                    carb.log_info(f"Arena rendered-frame capture requested: {screenshot_path}")

                pending_actions = run_after_tick_queue.qsize()
                for _ in range(pending_actions):
                    try:
                        deferred_action = run_after_tick_queue.get_nowait()
                    except queue.Empty:
                        break

                    try:
                        deferred_action()
                    except Exception as e:
                        carb.log_error(f"Deferred action failed: {e}\n{traceback.format_exc()}")

    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt, shutting down.')
    except Exception as e:
        controller.get_logger().error(f'Exception in main loop: {e}')
        controller.get_logger().error(traceback.format_exc())
        traceback.print_exc(file=sys.stdout)
    finally:
        controller.get_logger().info('Shutting down ROS 2 node and simulation.')
        controller.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


# =================================================================================
if __name__ == "__main__":
    main()
