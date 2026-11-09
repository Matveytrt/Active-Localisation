"""Run the unchanged detector with source parameters and measured printed size."""
import argparse
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from world_source import audit

parser = argparse.ArgumentParser()
parser.add_argument('family', choices=('HD15', 'HD17'))
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
source = repo/f'navigation/src/stag_ros2_enhanced/config/detector_params_{args.family}.yaml'
params = yaml.safe_load(source.read_text())['/**']['ros__parameters']
markers = [m for m in audit(repo)['markers'] if m['family'] == args.family]
sizes = {round(edge, 9) for marker in markers for edge in marker['detected_edge_m']}
assert len(sizes) == 1, 'One detector requires one physical black-square size'
params['marker_size'] = sizes.pop()
params['use_sim_time'] = True
output = Path('/workspace/stag')
output.mkdir(exist_ok=True)
config = output/f'{args.family}.yaml'
config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
print(f'{args.family}: original detector, physical black-square edge {params["marker_size"]} m', flush=True)
executable = str(Path(get_package_prefix('stag_ros2_enhanced'))/'lib/stag_ros2_enhanced/stag_detector_node')
os.execv(executable, [executable, '--ros-args',
    '-r', '__node:=stag_'+args.family.lower(), '-r', '__ns:=/isaac/stag', '--params-file', str(config)])
