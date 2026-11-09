"""Read-only graph/provenance acceptance while the baseline stack is alive."""
import hashlib
import argparse
import json
from pathlib import Path
import time
import rclpy
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser()
parser.add_argument('--db', type=Path, default=Path('/workspace/maps/isaac-20261109-global.db'))
args = parser.parse_args()
rclpy.init(args=[])
node = rclpy.create_node('isaac_navigation_audit')
edges = set()
node.create_subscription(TFMessage, '/tf', lambda m: edges.update((t.header.frame_id, t.child_frame_id) for t in m.transforms), 100)
end = time.monotonic()+4
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.02)
owners = {topic: sorted(p.node_name for p in node.get_publishers_info_by_topic(topic))
          for topic in ('/tf', '/map', '/odom_gt', '/odom', '/landmark_detections', '/cmd_vel')}
assert ('map', 'odom') in edges and ('odom', 'base_link') in edges
assert {(a, b) for a, b in edges if b == 'base_link'} == {('odom', 'base_link')}
assert owners['/map'] == ['rtabmap'] and owners['/odom'] == ['odom_noise_node']
assert owners['/odom_gt'] == ['isaac_astribot_bridge']
assert owners['/landmark_detections'] == ['tf_to_fiducial_node']
assert owners['/tf'].count('rtabmap') == owners['/tf'].count('odom_noise_node') == 1
assert set(owners['/cmd_vel']) <= {'velocity_smoother', 'behavior_server'}, owners['/cmd_vel']
db = args.db
digest = hashlib.sha256(db.read_bytes()).hexdigest()
assert digest == db.with_suffix('.db.sha256').read_text()
result = {'status': 'passed', 'edges': sorted(edges), 'owners': owners, 'pristine_sha256': digest}
Path('/workspace/baseline-007/graph-audit.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
