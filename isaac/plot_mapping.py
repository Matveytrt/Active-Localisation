"""Measured map / trajectory figure. Source and GT appear only as evaluation overlays."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--prefix', type=Path, required=True, help='Saved occupancy-map prefix, without suffix')
parser.add_argument('--landmarks', type=Path, required=True)
args = parser.parse_args()
mapping = Path('/workspace/mapping-007')
route = json.loads((mapping/'result.json').read_text())
world_map = np.array(route['initial_world_from_map'])
trajectory = np.genfromtxt(mapping/'trajectory.csv', names=True, delimiter=',')
valid = np.isfinite(trajectory['map_x']) & np.isfinite(trajectory['map_y'])
trajectory = trajectory[valid]
truth = np.column_stack([trajectory['gt_x'], trajectory['gt_y'], np.ones(len(trajectory))]) @ np.linalg.inv(world_map).T
estimate = np.column_stack([trajectory['map_x'], trajectory['map_y']])
odom = np.column_stack([trajectory['odom_x'], trajectory['odom_y'], np.ones(len(trajectory))]) @ np.linalg.inv(world_map).T
metrics = {'sampled_mapping_xy_rmse_m': float(np.sqrt(np.mean(np.sum((estimate-truth[:, :2])**2, axis=1)))),
           'sampled_odom_xy_rmse_m': float(np.sqrt(np.mean(np.sum((odom[:, :2]-truth[:, :2])**2, axis=1)))),
           'samples': len(trajectory), 'note': 'One mapping pilot, initial-frame alignment; not paired APE/RPE experiments'}
(mapping/'trajectory-evaluation.json').write_text(json.dumps(metrics, indent=2))
meta = yaml.safe_load(args.prefix.with_suffix('.yaml').read_text())
image = np.asarray(Image.open(args.prefix.with_suffix('.pgm')))
x, y = meta['origin'][:2]
res = meta['resolution']
extent = [x, x+image.shape[1]*res, y, y+image.shape[0]*res]
fig, ax = plt.subplots(figsize=(8, 8))
ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
ax.plot(truth[:, 0], truth[:, 1], color='#2563eb', lw=2, label='GT (evaluation only)')
ax.plot(estimate[:, 0], estimate[:, 1], color='#16a34a', lw=1, label='SLAM estimate')
for item in yaml.safe_load(args.landmarks.read_text())['big_markers']:
    p = item['position']
    ax.scatter([p['x']], [p['y']], marker='s', color='#dc2626', s=35)
    ax.annotate(str(item['id']), [p['x']+.08, p['y']+.08], color='#991b1b')
ax.scatter([], [], marker='s', color='#dc2626', label='Optimized real STag landmarks')
ax.set(xlabel='map x [m]', ylabel='map y [m]', title='Isaac: sensor-built RTAB-Map and measured trajectory')
ax.set_aspect('equal')
ax.legend(loc='upper left', fontsize=8)
fig.tight_layout()
fig.savefig(mapping/'map-and-trajectory.png', dpi=150)
print(json.dumps(metrics))
