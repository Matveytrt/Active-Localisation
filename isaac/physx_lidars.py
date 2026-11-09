"""Native batched PhysX Generic Sensors with the original Gazebo ray pattern."""
import numpy as np
import omni.kit.commands
from pxr import Gf
from isaacsim.sensors.physx import _range_sensor
from bridge_core import beam_directions, quaternion_xyzw, physx_generic_angles


class PhysXLidars:
    def __init__(self, specs, frames):
        self.interface = _range_sensor.acquire_generic_sensor_interface()
        self.paths, self.patterns, self.beams = {}, {}, {}
        for side in ('front', 'back'):
            spec = specs[side]
            transform = frames['lidar_'+side]
            quat = quaternion_xyzw(transform[:3, :3])
            path = '/World/Robot/Lidar'+side.capitalize()
            ok, _ = omni.kit.commands.execute('RangeSensorCreateGeneric', path=path,
                parent=None, translation=Gf.Vec3d(*transform[:3, 3].tolist()),
                orientation=Gf.Quatd(float(quat[3]), Gf.Vec3d(*quat[:3].tolist())),
                min_range=spec['min'], max_range=spec['max'], draw_points=False,
                draw_lines=False, sampling_rate=120*16*60)
            assert ok, 'Could not create '+path
            self.paths[side] = path
            self.beams[side] = beam_directions(spec)
            pattern = physx_generic_angles(spec)
            # Exactly one full pattern per 60 Hz physics tick. Publishing remains
            # 10 Hz; the native batch avoids 3840 Python/C++ calls per scan.
            self.patterns[side] = np.tile(pattern.astype(np.float32), (1, 120))

    def refill(self):
        for side, path in self.paths.items():
            if self.interface.send_next_batch(path):
                self.interface.set_next_batch_rays(path, self.patterns[side])

    def depths(self, side):
        return np.asarray(self.interface.get_linear_depth_data(self.paths[side])).reshape(-1).copy()

    def ready(self):
        return all(len(self.depths(side)) == len(self.beams[side]) for side in self.paths)
