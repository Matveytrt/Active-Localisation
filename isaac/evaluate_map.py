"""Evaluate actual exported optimized landmarks; never author a replacement map."""
import json
import argparse
from pathlib import Path
import sqlite3
import numpy as np
import yaml
from world_source import audit

parser = argparse.ArgumentParser()
parser.add_argument('--db', type=Path, required=True)
parser.add_argument('--route', type=Path, default=Path('/workspace/mapping-007/result.json'))
args = parser.parse_args()
db = args.db
landmarks = yaml.safe_load(db.with_suffix('.landmarks.yaml').read_text())
assert landmarks['source'] == 'optimized_graph', 'Raw-odometry fallback is not accepted'
route = json.loads(args.route.read_text())
assert route['status'] == 'passed'
world_map = np.array(route['initial_world_from_map'])
source = audit(Path('/repo'))
errors = []
for item in landmarks['big_markers']:
    marker_id = item['id']
    assert marker_id in (1, 2, 3, 4, 5)
    p = item['position']
    observed = world_map @ [p['x'], p['y'], 1.]
    expected = next(m for m in source['markers'] if m['family'] == 'HD17' and m['id'] == marker_id)
    expected = (np.array(expected['transform']) @ [0, 0, expected['size'][2]/2+.00001, 1])[:2]
    errors.append({'id': marker_id, 'world_xy': observed[:2].tolist(), 'expected_world_xy': expected.tolist(),
                   'error_m': float(np.linalg.norm(observed[:2]-expected))})
con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
count = con.execute('select count(*) from Node').fetchone()[0]
landmark_links = con.execute('select count(*) from Link where type=8').fetchone()[0]
con.close()
assert count >= 50 and len(errors) >= 2
median = float(np.median([e['error_m'] for e in errors]))
assert median < .20, errors
result = {'status': 'passed', 'nodes': count, 'landmark_links': landmark_links,
    'source': landmarks['source'], 'landmarks': errors, 'median_xy_error_m': median}
db.with_suffix('.evaluation.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
