import math
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bridge_core import PlanarBase, ROOM_BOXES, integrate, yaw_matrix, quaternion_xyzw, sensor_specs, beam_directions, physx_generic_angles


def test_body_axes_and_exact_circular_motion():
    assert np.allclose(integrate([0, 0, math.pi/2], [1, .5, 0], 2), [-1, 2, math.pi/2])
    assert np.allclose(integrate([0, 0, 0], [1, 0, 1], math.pi/2), [1, 1, math.pi/2])


def test_collision_stops_before_scene_wall_without_tunnelling():
    base = PlanarBase(ROOM_BOXES)
    base.accept([.6, 0, 0], 0, 0)
    base.step(20, 0, 0)
    assert base.blocked and 3.65 < base.pose[0] <= 3.675
    assert np.allclose(base.actual, 0)


def test_watchdogs_nonfinite_commands_and_reset():
    base = PlanarBase([])
    base.accept([.2, .1, .3], 0, 0)
    base.step(.1, .1, .1)
    pose = base.pose.copy()
    base.step(.1, .6, .2)
    assert np.array_equal(base.pose, pose)
    base.accept([.2, 0, 0], 1, 1)
    base.step(.1, 1.1, 3.1)
    assert np.array_equal(base.pose, pose)
    assert not base.accept([float('nan'), 0, 0], 1, 1)
    base.reset()
    assert np.array_equal(base.pose, np.zeros(3))


def test_quaternions_cover_sensor_rotations():
    for angles in ((0, 0, 0), (math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi), (.4, -.8, 2.7)):
        matrix = Rotation.from_euler('xyz', angles).as_matrix()
        assert np.allclose(Rotation.from_quat(quaternion_xyzw(matrix)).as_matrix(), matrix)


def test_scan_pattern_comes_from_source_xacros():
    package = Path(__file__).resolve().parents[1] / 'gazebo/src/world_and_robot'
    specs = sensor_specs(package)
    for side in ('front', 'back'):
        directions = beam_directions(specs[side])
        assert directions.shape == (1920, 3)
        assert np.allclose(np.linalg.norm(directions, axis=1), 1)
        assert np.isclose(directions[0, 2], math.sin(-.3))
        assert specs[side]['hz'] == 10
    assert specs['camera']['width'] == 640
    assert specs['camera']['height'] == 480
    assert specs['camera']['hfov'] == 1.089


def test_physx_zenith_preserves_gazebo_elevation_and_ray_order():
    package = Path(__file__).resolve().parents[1] / 'gazebo/src/world_and_robot'
    for side in ('front', 'back'):
        spec = sensor_specs(package)[side]
        azimuth, zenith = physx_generic_angles(spec)
        # Convention measured against native get_point_cloud_data, not inferred
        # from the SDK name: negative zenith gives a positive local z.
        sdk_directions = np.column_stack([np.cos(zenith)*np.cos(azimuth),
            np.cos(zenith)*np.sin(azimuth), -np.sin(zenith)])
        assert np.allclose(sdk_directions, beam_directions(spec), atol=1e-7)
        assert zenith[0] > 0 and zenith[-1] < 0


def test_rotated_thin_wall_and_nonzero_spawn_reset():
    wall = {'center': [2., 0., 1.], 'size': [3., .1, 2.], 'rotation': yaw_matrix(math.pi/2)}
    base = PlanarBase([wall], spawn=(-1., -.5, 0.))
    base.accept([.6, 0, 0], 0, 0)
    base.step(20, 0, 0)
    assert base.blocked and 1.61 < base.pose[0] <= 1.625
    base.reset()
    assert np.array_equal(base.pose, [-1., -.5, 0.])
