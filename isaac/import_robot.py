"""Import our own fixed-pose Astribot URDF and capture an actual rendered frame."""
import json
import traceback
import time
import sys
from pathlib import Path
from isaacsim import SimulationApp

repo = Path(__file__).resolve().parents[1]
output = repo / "isaac/runtime/robot"
output.mkdir(parents=True, exist_ok=True)
result_path = output / "import_result.json"
result_path.unlink(missing_ok=True)
sys.argv.extend(["--/renderer/enabled=rtx", "--/renderer/active=rtx"])
app = SimulationApp({"headless": True, "active_gpu": 0, "multi_gpu": False,
                     "width": 640, "height": 480})
try:
    import numpy as np
    import omni.kit.commands
    import omni.usd
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom, UsdLux
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.sensors.camera import Camera
    from prepare_robot import prepare

    # Kit announces "app ready" before asynchronous RTX initialization completes.
    # A fixed number of fast physics updates can finish before the first render.
    deadline = time.monotonic() + 180
    context = omni.usd.get_context()
    while "rtx" not in context.get_attached_hydra_engine_names():
        if time.monotonic() > deadline:
            raise TimeoutError("RTX engine did not attach within 180 seconds")
        app.update()
        time.sleep(0.05)
    print("AL_RTX_READY", flush=True)
    urdf, metadata = prepare(repo)
    enable_extension("isaacsim.asset.importer.urdf")
    app.update()
    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    assert ok
    config.merge_fixed_joints = False
    config.fix_base = True
    config.import_inertia_tensor = True
    config.distance_scale = 1.0
    config.self_collision = False
    config.convex_decomp = False
    config.make_default_prim = True
    asset_dir = output / "asset"
    asset_dir.mkdir(exist_ok=True)
    usd_path = asset_dir / "astribot.usd"
    ok, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=str(urdf),
        import_config=config, dest_path=str(usd_path))
    assert ok and usd_path.exists(), f"URDF import failed: {prim_path}"
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    robot = UsdGeom.Xform.Define(stage, "/World/Astribot").GetPrim()
    robot.GetReferences().AddReference(str(usd_path), prim_path)
    app.update()
    app.reset_render_settings()
    mesh_prims = [p for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
                  if p.IsA(UsdGeom.Mesh)]
    assert mesh_prims, "No mesh geometry imported"
    frames = {p.GetName() for p in stage.Traverse()}
    for frame in ("base_link", "torso_rgbd", "lidar_front", "lidar_back"):
        assert frame in frames, f"Frame missing after import: {frame}"
    light = UsdLux.DomeLight.Define(stage, "/World/PreviewLight")
    light.CreateIntensityAttr(150)
    position = Gf.Vec3d(2.5, -2.5, 1.8)
    view = Gf.Matrix4d().SetLookAt(position, Gf.Vec3d(0, 0, 0.8), Gf.Vec3d(0, 0, 1))
    quat = view.GetInverse().ExtractRotationQuat()
    orientation = np.array([quat.GetReal(), *quat.GetImaginary()])
    camera = Camera("/World/PreviewCamera", position=np.array(position),
                    orientation=orientation, resolution=(640, 480))
    world = World(stage_units_in_meters=1.0)
    world.scene.add(FixedCuboid(
        prim_path="/World/PreviewFloor", position=np.array([0.0, 0.0, -0.20]),
        scale=np.array([20.0, 20.0, 0.08]), color=np.array([0.12, 0.15, 0.20])))
    world.reset()
    camera.initialize()
    camera.set_world_pose(position=np.array(position), orientation=orientation, camera_axes="usd")
    camera.set_focal_length(camera.get_horizontal_aperture() / (2 * np.tan(np.deg2rad(55.0) / 2)))
    deadline = time.monotonic() + 180
    rgba = None
    while time.monotonic() < deadline:
        world.step(render=True)
        rgba = camera.get_rgba()
        if rgba is not None and rgba.shape == (480, 640, 4):
            break
        time.sleep(0.02)
    assert rgba is not None and rgba.shape == (480, 640, 4), f"Invalid camera frame: {getattr(rgba, 'shape', None)}"
    assert np.std(rgba[:, :, :3]) > 5, "Rendered frame is blank"
    Image.fromarray(rgba.astype(np.uint8)).save(output / "astribot_preview.png")
    stage.GetRootLayer().Export(str(output / "preview_scene.usd"))
    result = {"status": "passed", "robot_prim": prim_path,
              "urdf_links": metadata["links"], "urdf_mesh_files": metadata["meshes"],
              "usd_mesh_prims": len(mesh_prims), "image": "astribot_preview.png"}
    result_path.write_text(json.dumps(result, indent=2))
    print("AL_ROBOT_IMPORT_PASSED " + json.dumps(result), flush=True)
except BaseException as exc:
    print("AL_ROBOT_IMPORT_FAILED " + traceback.format_exc(), flush=True)
    result_path.write_text(json.dumps({"status": "failed", "error": repr(exc)}, indent=2))
    raise
finally:
    app.close()
