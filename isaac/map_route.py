"""Build an own sensor map with estimated-pose teleoperation; log GT only for evaluation."""
import json
import math
from pathlib import Path
import traceback
import numpy as np
import rclpy
from slam_probe import Probe, matrix, planar, stamp

output = Path('/workspace/mapping-007')
output.mkdir(exist_ok=True)
result = {'status': 'running', 'checks': {}, 'waypoints': []}
(output/'result.json').write_text(json.dumps(result, indent=2))
rclpy.init()
node = Probe('isaac_mapping_driver', output)
try:
    node.wait(lambda: node.gt is not None and node.odom is not None and node.estimate() is not None
              and node.grid is not None and node.info is not None, timeout=360)
    initial_world_map = matrix(planar(node.gt.pose.pose)) @ np.linalg.inv(matrix(node.estimate()))
    result['initial_world_from_map'] = initial_world_map.tolist()
    node.move([.2, .1, math.pi/2])
    node.hold(2.)
    node.wait(lambda: 1 in node.landmark_ids and 'stag_hd17_1' in node.marker_tf, timeout=15)
    landmark = node.landmarks[1]
    assert landmark.header.frame_id == 'base_link'
    result['checks']['nonzero_landmark'] = {'id': landmark.id, 'frame': landmark.header.frame_id,
        'position': [landmark.pose.pose.position.x, landmark.pose.pose.position.y, landmark.pose.pose.position.z]}
    # Evaluate the observation against source geometry after stopping. This GT
    # comparison does not feed the waypoint controller or RTAB-Map.
    from scipy.spatial.transform import Rotation
    from world_source import audit
    gt = node.gt.pose.pose
    world_base = np.eye(4)
    world_base[:3, :3] = Rotation.from_quat([gt.orientation.x, gt.orientation.y, gt.orientation.z, gt.orientation.w]).as_matrix()
    world_base[:3, 3] = [gt.position.x, gt.position.y, gt.position.z]
    expected = next(m for m in audit(Path('/repo'))['markers'] if m['family'] == 'HD17' and m['id'] == 1)
    face = np.array(expected['transform'])
    center = face @ [0, 0, expected['size'][2]/2+.00001, 1]
    observed = world_base @ [*result['checks']['nonzero_landmark']['position'], 1]
    error = float(np.linalg.norm(observed[:3]-center[:3]))
    result['checks']['nonzero_landmark']['world_position_error_m'] = error
    assert error < .05, error
    print('NONZERO_LANDMARK_PASSED', result['checks']['nonzero_landmark'], flush=True)
    node.move([.2, .1, -math.pi/2])
    node.hold(1.)
    since = stamp(node.gt)
    node.hold(1.)
    assert not any(ids for t, ids in node.landmark_messages if t > since), 'stale landmarks remain visible'
    result['checks']['landmark_age_gate'] = True
    route = [[.2, .1, math.pi/2], [1.9, .1, math.pi/2], [2.65, .1, math.pi/2],
        [2.65, 2.6, math.pi/2], [1.9, 2.6, math.pi/2], [.2, 2.6, math.pi/2],
        [-1.5, 2.6, math.pi/2], [-3.8, 2.6, math.pi/2], [-3.8, 4.7, math.pi/2],
        [2.65, 4.7, math.pi/2], [2.65, -.5, math.pi/2], [-2., -.5, math.pi/2], [-2., -.5, 0.]]
    for goal in route:
        estimate = node.move(goal)
        result['waypoints'].append({'goal': goal, 'estimate': estimate, 'gt': planar(node.gt.pose.pose).tolist()})
        print('MAPPING_WAYPOINT', goal, estimate, 'landmarks', sorted(node.landmark_ids), flush=True)
        (output/'result.json').write_text(json.dumps(result, indent=2))
    node.hold(3.)
    assert node.landmark_ids and node.landmark_ids <= {1, 2, 3, 4, 5}, node.landmark_ids
    assert len(node.landmark_ids) >= 2
    grid = np.array(node.grid.data)
    assert np.sum(grid == 0) > 1000 and np.sum(grid >= 65) > 100
    result['checks']['map_grid'] = {'width': node.grid.info.width, 'height': node.grid.info.height,
        'resolution': node.grid.info.resolution, 'free_cells': int(np.sum(grid == 0)), 'occupied_cells': int(np.sum(grid >= 65))}
    result['checks']['raw_ids'] = sorted(node.raw_ids)
    result['checks']['allowed_landmark_ids'] = sorted(node.landmark_ids)
    result['checks']['slam_ref_id'] = node.info.ref_id
    assert node.info.ref_id >= 50, node.info.ref_id
    result['status'] = 'passed'
    print('AL_MAPPING_ROUTE_PASSED', flush=True)
except BaseException:
    result['status'], result['error'] = 'failed', traceback.format_exc()
    print(result['error'], flush=True)
    raise
finally:
    node.send([0, 0, 0])
    (output/'result.json').write_text(json.dumps(result, indent=2))
    node.close()
    rclpy.shutdown()
