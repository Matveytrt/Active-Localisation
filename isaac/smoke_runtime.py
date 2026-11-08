"""Check the stock Isaac runtime, physics and Humble /clock without team code."""
import json
import time
import traceback
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 640, "height": 480})
result_path = Path(__file__).resolve().parent / "runtime/smoke_result.json"
result_path.parent.mkdir(parents=True, exist_ok=True)
result_path.unlink(missing_ok=True)
try:
    import numpy as np
    import omni.graph.core as og
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    app.update()
    world = World(stage_units_in_meters=1.0)
    world.scene.add(FixedCuboid(
        prim_path="/World/Floor", position=np.array([0.0, 0.0, -0.1]),
        scale=np.array([10.0, 10.0, 0.2]), color=np.array([0.3, 0.3, 0.3])))
    cube = world.scene.add(DynamicCuboid(
        prim_path="/World/Cube", position=np.array([0.0, 0.0, 1.5]),
        scale=np.array([0.5, 0.5, 0.5]), color=np.array([0.1, 0.5, 0.9])))
    keys = og.Controller.Keys
    og.Controller.edit({"graph_path": "/ClockGraph", "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.SET_VALUES: [("Context.inputs:useDomainIDEnvVar", True),
                          ("Clock.inputs:topicName", "/clock")],
        keys.CONNECT: [("Tick.outputs:tick", "Clock.inputs:execIn"),
                       ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
                       ("Context.outputs:context", "Clock.inputs:context")],
    })
    world.reset()
    started = time.monotonic()
    steps = 0
    print("AL_SMOKE_READY", flush=True)
    while time.monotonic() - started < 30.0:
        world.step(render=True)
        steps += 1
    position, _ = cube.get_world_pose()
    assert steps >= 120, f"Too few physics steps: {steps}"
    assert 0.20 < position[2] < 0.35, f"Unexpected resting height: {position}"
    result = {"status": "passed", "steps": steps,
              "wall_seconds": time.monotonic() - started,
              "cube_position": position.tolist(),
              "clock_topic": "/clock", "ros_backend": "humble"}
    result_path.write_text(json.dumps(result, indent=2))
    print("AL_SMOKE_PASSED " + json.dumps(result), flush=True)
except BaseException as exc:
    print("AL_SMOKE_FAILED " + traceback.format_exc(), flush=True)
    result_path.write_text(json.dumps({"status": "failed", "error": repr(exc)}, indent=2))
    raise
finally:
    app.close()
