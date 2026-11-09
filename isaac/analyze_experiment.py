"""Stage008 paired pilot audit. Route/yaw effect is not statistical superiority."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch')
    args = parser.parse_args()
    batch = Path(args.batch)
    manifest = json.loads((batch/'manifest.json').read_text())
    assert manifest['status'] == 'passed', manifest['status']
    metrics, trajectories = {}, {}
    fig, axes = plt.subplots(2, 1, figsize=(10, 9))
    for episode in manifest['episodes']:
        mode = episode['mode']
        folder = batch/(mode+'_01')
        acceptance = json.loads((folder/'acceptance.json').read_text())
        assert acceptance['status'] == 'passed' and episode['clean_shutdown']
        result = yaml.safe_load((folder/'result.yaml').read_text())
        record = {'samples': {}, 'goals': acceptance['goals'],
                  'navigation_rtf': acceptance['navigation_rtf']}
        values = {}
        for name in ('gt', 'est', 'map_odom', 'noisy'):
            data = np.loadtxt(folder/(name+'.tum'), comments='#')
            assert data.ndim == 2 and data.shape[1] == 8 and len(data) > 10, (mode, name)
            assert np.isfinite(data).all() and np.all(np.diff(data[:, 0]) > 0), (mode, name)
            assert np.max(np.abs(np.linalg.norm(data[:, 4:8], axis=1)-1)) < .001
            record['samples'][name] = len(data)
            values[name] = data
        goal = result['goals'][1]
        gt = values['gt']
        selection = (gt[:, 0] >= goal['start_sim_s']) & (gt[:, 0] <= goal['end_sim_s'])
        leg = gt[selection]
        yaw = np.arctan2(2*(leg[:, 7]*leg[:, 6]+leg[:, 4]*leg[:, 5]), 1-2*(leg[:, 5]**2+leg[:, 6]**2))
        trajectories[mode] = np.column_stack([leg[:, 1:3], yaw])
        axes[0].plot(leg[:, 1], leg[:, 2], label=mode)
        axes[1].plot(leg[:, 1], np.rad2deg(yaw), '.', ms=2, label=mode)
        record['plans'] = sum(1 for _ in (folder/'plans.jsonl').open())
        assert record['plans'] > 0
        record['recoveries'] = [goal['recoveries'] for goal in result['goals']]
        metrics[mode] = record
    assert set(trajectories) == {'baseline', 'active', 'control'}
    # Common spatial bins avoid overweighting slower turns/standing samples.
    bins = np.arange(-1., 1.51, .1)
    records = []
    for x in bins[:-1]:
        a, c = [data[(data[:, 0] >= x) & (data[:, 0] < x+.1)]
                for data in (trajectories['active'], trajectories['control'])]
        assert len(a) and len(c), ('Missing common spatial bin', float(x))
        ay, cy = np.median(a[:, 1]), np.median(c[:, 1])
        ya = np.arctan2(np.sin(a[:, 2]).mean(), np.cos(a[:, 2]).mean())
        yc = np.arctan2(np.sin(c[:, 2]).mean(), np.cos(c[:, 2]).mean())
        records.append([float(x), float(ay-cy), float(abs(np.arctan2(np.sin(ya-yc), np.cos(ya-yc))))])
    delta_y = abs(float(np.mean([r[1] for r in records])))
    delta_yaw = float(np.mean([r[2] for r in records]))
    effect = delta_y >= .10 or delta_yaw >= .20
    result = {'status': 'passed' if effect else 'failed', 'modes': metrics,
        'effect': {'mean_route_y_difference_m': delta_y, 'mean_abs_heading_difference_rad': delta_yaw,
                   'bins_x_delta_y_abs_delta_yaw': records},
        'limitation': 'One paired pilot; MPPI RNG uncontrolled; not APE/RPE or statistical benefit.'}
    (batch/'analysis.json').write_text(json.dumps(result, indent=2))
    axes[0].set(xlabel='GT world x (m)', ylabel='GT world y (m)', title='Original south-wall scenario: goal 2')
    axes[0].axis('equal')
    axes[1].set(xlabel='GT world x (m)', ylabel='GT robot yaw (degrees)')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(batch/'comparison.png', dpi=150)
    print(json.dumps(result, indent=2))
    assert effect, 'Active/control route and heading did not meet the predeclared effect criterion'


if __name__ == '__main__':
    main()
