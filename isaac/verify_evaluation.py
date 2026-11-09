"""Analytical controls of the unchanged crop + evo pipeline, remote evaluation venv."""
import json
import argparse
import os
from pathlib import Path
import subprocess
import zipfile
import numpy as np
import yaml


def stats(folder, name):
    with zipfile.ZipFile(folder/'evo_none'/(name+'.zip')) as archive:
        return json.loads(archive.read('stats.json'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/workspace/evaluation-controls-009')
    root = Path(parser.parse_args().output)
    root.mkdir(exist_ok=False)
    result = {}
    for name, offset in [('identity', 0.), ('translation', .2)]:
        folder = root/name
        folder.mkdir()
        gt = np.array([[t, .6*t, 0, .16, 0, 0, 0, 1] for t in np.arange(0,4.01,.02)])
        est = np.array([[t, .6*t+offset, 0, 0, 0, 0, 0, 1] for t in np.arange(.005,4.,.1)])
        np.savetxt(folder/'gt.tum', gt, fmt='%.9f')
        np.savetxt(folder/'est.tum', est, fmt='%.9f')
        (folder/'result.yaml').write_text(yaml.safe_dump({'goals': [{'start_sim_s': .5, 'end_sim_s': 3.5}]}))
        proc = subprocess.run(['bash', '/repo/eval/run_evo.sh', str(folder), '--align', 'none',
                               '--plot_format', 'none'], text=True, capture_output=True,
                              env=dict(os.environ, PATH='/opt/al-eval/bin:'+os.environ['PATH']))
        (folder/'pipeline.log').write_text(proc.stdout+proc.stderr)
        assert proc.returncode == 0, proc.stdout+proc.stderr
        ape, rpe = stats(folder, 'ape'), stats(folder, 'rpe')
        assert abs(ape['rmse']-offset) < 1e-7, ape
        assert rpe['rmse'] < 1e-7, rpe
        window = yaml.safe_load((folder/'eval_window.yaml').read_text())
        assert window['source'] == 'result.yaml', window
        result[name] = {'ape_rmse_m': ape['rmse'], 'rpe_rmse_m': rpe['rmse'], 'window': window}
    (root/'result.json').write_text(json.dumps({'status':'passed','controls':result},indent=2))
    print('AL_EVALUATION_CONTROLS_PASSED', flush=True)


if __name__ == '__main__':
    main()
