"""Audit and evaluate every planned job, retaining failed jobs and paired units."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import zipfile
import numpy as np
from scipy.stats import t as student_t
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/repo/eval')
sys.path.insert(0, '/repo/isaac')
import summarize as original
import crop_run
from experiment_config import noise_parameters, nav_parameters, SLAM_RESOURCES

METRICS = ('ape_rmse', 'ape_max', 'rpe_rmse', 'duration_s', 'path_length_m',
           'det_total', 'det_per_m', 'det_coverage_pct', 'path_detection_pct',
           'longest_unobserved_path_m', 'recoveries', 'navigation_rtf')


def path_observation(gt, stamps, window=1.):
    """Integrate distance over union of recent-detection time intervals."""
    ts, xy = gt[:, 0], gt[:, 1:3]
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    intervals = []
    for stamp in sorted(stamps):
        low, high = max(ts[0], stamp), min(ts[-1], stamp+window)
        if high <= low:
            continue
        if intervals and low <= intervals[-1][1]:
            intervals[-1][1] = max(intervals[-1][1], high)
        else:
            intervals.append([low, high])
    covered, gap_start, gaps = 0., ts[0], []
    for low, high in intervals:
        covered += float(np.interp(high, ts, distance)-np.interp(low, ts, distance))
        gaps.append(float(np.interp(low, ts, distance)-np.interp(gap_start, ts, distance)))
        gap_start = high
    gaps.append(float(distance[-1]-np.interp(gap_start, ts, distance)))
    return 100*covered/distance[-1] if distance[-1]>0 else math.nan, max(gaps)


def digest_files(folder):
    h = hashlib.sha256()
    for name in ('gt.tum', 'est.tum', 'result.yaml'):
        h.update((folder/name).read_bytes())
    return h.hexdigest()


def evaluate_job(job, folder):
    row = dict(job)
    row['folder'] = str(folder)
    row['evaluation_error'] = ''
    try:
        result = yaml.safe_load((folder/'result.yaml').read_text())
        row['goals_total'] = len(result['goals'])
        row['goals_succeeded'] = sum(g['status']=='succeeded' for g in result['goals'])
        row['goal_statuses'] = [g['status'] for g in result['goals']]
        row['recoveries'] = sum(g.get('recoveries') or 0 for g in result['goals'])
        acceptance = json.loads((folder/'acceptance.json').read_text())
        initial = acceptance['initialization']['initial_estimate']
        row['post_initialpose_xy_error_m'] = math.hypot(initial[0]+2.,initial[1]+.5)
        row['post_initialpose_yaw_error_rad'] = abs(initial[2])
        assert row['post_initialpose_xy_error_m'] <= .05 and row['post_initialpose_yaw_error_rad'] <= math.radians(2), 'Known initial pose did not settle'
        row['pose_acceptance'] = acceptance['status']
        row['strict_pose_goals_passed'] = sum(g['within_acceptance'] for g in acceptance['goals'])
        row['navigation_rtf'] = acceptance.get('navigation_rtf')
        row['max_goal_gt_error_m'] = max(g['gt_xy_error_m'] for g in acceptance['goals'])
        row['max_goal_estimated_yaw_error_rad'] = max(g['estimated_yaw_error_rad'] for g in acceptance['goals'])
        row['max_goal_estimated_xy_error_m'] = max(g['estimated_xy_error_m'] for g in acceptance['goals'])
        samples={}
        for name in ('gt','est','noisy','map_odom'):
            data = np.loadtxt(folder/(name+'.tum'), comments='#')
            assert data.ndim==2 and data.shape[1]==8 and len(data)>10, name
            assert np.isfinite(data).all() and np.all(np.diff(data[:,0])>0), name
            assert np.max(np.abs(np.linalg.norm(data[:,4:8],axis=1)-1))<.001, name
            samples[name]=data
        meta=yaml.safe_load((folder/'meta.yaml').read_text())
        for name,counter in [('gt','gt_written'),('est','est_written'),('map_odom','map_odom_written')]:
            assert meta['counters'][counter]==len(samples[name]), (name,'metadata/file count mismatch')
        goal_end=max(g['end_sim_s'] for g in result['goals'] if g.get('end_sim_s') is not None)
        assert meta['stop_ros_time']>=goal_end, 'Recorder finalized before goal window ended'
        assert samples['gt'][-1,0]>=goal_end-.04, 'GT did not cover the end of the goal window'
        stamp = digest_files(folder)
        cache = folder/'evaluation-input.sha256'
        outputs = [folder/'evo_none'/(name+'.zip') for name in ('ape','rpe','ape_rot','rpe_rot')]
        if not cache.exists() or cache.read_text().strip()!=stamp or not all(p.exists() for p in outputs):
            proc = subprocess.run(['bash','/repo/eval/run_evo.sh',str(folder),'--align','none',
                                   '--plot_format','none'],text=True,capture_output=True,
                                  env=dict(os.environ,PATH='/opt/al-eval/bin:'+os.environ['PATH']))
            (folder/'evaluation.log').write_text(proc.stdout+proc.stderr)
            assert proc.returncode==0, 'evo pipeline failed'
            cache.write_text(stamp+'\n')
        metrics = original.load_run(str(folder), align='none')
        assert metrics['window_source']=='result.yaml', metrics['notes']
        assert all(math.isfinite(metrics[k]) for k in ('ape_rmse','ape_max','rpe_rmse','ape_rot_rmse','rpe_rot_rmse')), metrics['notes']
        row.update(metrics)
        row['ape_samples'] = original._read_res(str(folder/'evo_none/ape.zip'))['n']
        row['rpe_pairs'] = original._read_res(str(folder/'evo_none/rpe.zip'))['n']
        row['ape_match_fraction'] = row['ape_samples']/len(np.loadtxt(folder/'est_eval.tum',comments='#'))
        row.update(mode=job['mode'], seed=job['seed'], noise_scale=job['noise_scale'],
                   run=job['batch'], status=job['status'])
        gt = np.loadtxt(folder/'gt_eval.tum',comments='#')
        t0,t1 = metrics['window_t0'],metrics['window_t1']
        gt = gt[(gt[:,0]>=t0)&(gt[:,0]<=t1)]
        lm = original.read_landmarks_csv(str(folder/'landmarks.csv'))
        assert meta['counters']['landmark_msgs_written']==len(lm), 'Landmark counter mismatch'
        row['landmark_ids'] = sorted({int(marker) for entry in lm for marker in entry[2]})
        assert set(row['landmark_ids']) <= {1,2,3,4,5}, row['landmark_ids']
        stamps = [r[0] for r in lm if r[1]>0 and t0<=r[0]<=t1]
        row['path_detection_pct'],row['longest_unobserved_path_m'] = path_observation(gt,stamps)
        row['audit'] = 'passed'
    except Exception as error:
        row.update(audit='failed',evaluation_error=str(error))
    return row


def finite(values):
    return [float(v) for v in values if v is not None and isinstance(v,(int,float)) and math.isfinite(v)]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('series')
    parser.add_argument('--partial',action='store_true')
    args=parser.parse_args()
    root=Path(args.series)
    manifest=json.loads((root/'manifest.json').read_text())
    if not args.partial:
        assert manifest['status']=='complete', manifest['status']
    rows=[]
    for job in manifest['jobs']:
        if job.get('status') not in ('passed','failed'):
            continue
        folder=Path('/workspace/experiments')/job['batch']/(job['mode']+'_01')
        row=evaluate_job(job,folder)
        try:
            batch=json.loads((folder.parent/'manifest.json').read_text())
            assert batch['map_sha256']==manifest['map_sha256']
            assert batch['seed']==job['seed'] and batch['noise_scale']==job['noise_scale']
            assert batch['git_head']==manifest['git_head'], 'Source commit changed during series'
            assert set(batch['uncommitted_sha256']) <= {'TASK_REGISTRY.md'}, 'Runtime source changed during series'
            assert len(batch['episodes'])==1
            entry=batch['episodes'][0]
            assert entry['initial_db_sha256']==entry['pristine_after_sha256']==manifest['map_sha256']
            containers=json.loads((folder/'containers.json').read_text())
            assert len(containers)==10
            teardown_problems=[]
            for name,record in containers.items():
                state=record['state']
                assert not state['OOMKilled'] and not state['Running'], name
                allowed=(0,1) if name=='al-isaac-episode' and job['status']=='failed' else (0,)
                if state['ExitCode'] not in allowed:
                    teardown_problems.append({'container':name,'exit_code':state['ExitCode']})
            row['teardown_audit']='failed' if teardown_problems else 'passed'
            row['teardown_problems']=teardown_problems
            row['runner_clean_shutdown_flag']=entry['clean_shutdown']
            parameters=json.loads((folder/'runtime-parameters.json').read_text())
            noise=noise_parameters('/repo',job['seed'],float(job['noise_scale']))['odom_noise_node']['ros__parameters']
            assert all(v==noise[k] for k,v in parameters['noise'].items())
            assert parameters['slam']==SLAM_RESOURCES
            landmarks=Path('/workspace/maps/isaac-20261109-global.landmarks.yaml')
            expected_nav,_=nav_parameters('/repo',job['mode'],landmarks)
            assert yaml.safe_load((folder/'config/nav.yaml').read_text())==expected_nav
            controller=expected_nav['controller_server']['ros__parameters']['FollowPath']
            assert all(v==controller[k.split('.',1)[1]] for k,v in parameters['controller'].items())
            planner=expected_nav['planner_server']['ros__parameters']['GridBased']
            assert all(v==planner[k.split('.',1)[1]] for k,v in parameters['planner'].items())
            if job['mode']!='baseline':
                layer=expected_nav['global_costmap']['global_costmap']['ros__parameters']['loc_layer']
                assert all(v==layer[k.split('.',1)[1]] for k,v in parameters['layer'].items())
                logs=(folder/'al-isaac-baseline.log').read_text()
                if job['mode']=='active':
                    assert 'scorer=marker_soft_count (5 markers)' in logs
                    assert 'layer costs computed for' in logs
                else:
                    assert 'scorer=null (0 markers)' in logs and '(disabled)' in logs
            assert batch['landmarks_sha256']==hashlib.sha256(landmarks.read_bytes()).hexdigest()
            scenario=Path('/repo/navigation/src/al_eval/config/scenarios/shelves_loop_a.yaml')
            assert batch['scenario_sha256']==hashlib.sha256(scenario.read_bytes()).hexdigest()
            ready=json.loads((folder/'bridge-ready.json').read_text())
            assert ready['seeds']=={'front':job['seed'],'back':job['seed']+1}
            assert ready['spawn']==[-2.,-.5,0.] and ready['base_height']==.16
            bridge=json.loads((folder/'bridge-result.json').read_text())
            assert bridge['scene']=='own-world' and all(v>0 for v in bridge['counts'].values())
            row['input_audit']='passed'
        except Exception as error:
            row['input_audit'],row['input_error']='failed',str(error)
        rows.append(row)
    groups,paired={},{}
    for scale in (4,6):
        groups[str(scale)]={}
        for mode in ('baseline','active','control'):
            subset=[r for r in rows if r['noise_scale']==scale and r['mode']==mode]
            stats={}
            for metric in METRICS:
                values=finite([r.get(metric) for r in subset])
                stats[metric]={'n':len(values),'mean':float(np.mean(values)) if values else None,
                    'std':float(np.std(values,ddof=1)) if len(values)>1 else None}
            stats['runs']=len(subset)
            stats['passed_jobs']=sum(r['status']=='passed' for r in subset)
            stats['goals_succeeded']=sum(r.get('goals_succeeded',0) for r in subset)
            stats['goals_total']=sum(r.get('goals_total',3) for r in subset)
            groups[str(scale)][mode]=stats
        paired[str(scale)]={}
        for reference in ('baseline','control'):
            comparisons={}
            for metric in METRICS:
                differences=[]
                seeds=[]
                for seed in range(42,47):
                    a=[r for r in rows if r['noise_scale']==scale and r['seed']==seed and r['mode']=='active']
                    b=[r for r in rows if r['noise_scale']==scale and r['seed']==seed and r['mode']==reference]
                    if a and b and finite([a[0].get(metric)]) and finite([b[0].get(metric)]):
                        differences.append(a[0][metric]-b[0][metric]);seeds.append(seed)
                n=len(differences)
                mean=float(np.mean(differences)) if n else None
                half=float(student_t.ppf(.975,n-1)*np.std(differences,ddof=1)/math.sqrt(n)) if n>1 else None
                comparisons[metric]={'n':n,'seeds':seeds,'differences':differences,'mean':mean,
                    'ci95':[mean-half,mean+half] if half is not None else None}
            paired[str(scale)]['active-minus-'+reference]=comparisons
    result={'status':'partial' if args.partial else 'complete','planned_jobs':len(manifest['jobs']),
        'evaluated_jobs':len(rows),'passed_jobs':sum(r['status']=='passed' for r in rows),
        'audit_failures':sum(r.get('audit')!='passed' or r.get('input_audit')!='passed' for r in rows),
        'teardown_failures':sum(r.get('teardown_audit')!='passed' for r in rows),
        'navigation_goals_succeeded':sum(r.get('goals_succeeded',0) for r in rows),
        'strict_pose_goals_passed':sum(r.get('strict_pose_goals_passed',0) for r in rows),
        'pose_acceptance_failures':sum(r.get('pose_acceptance')!='passed' for r in rows),
        'rows':rows,'groups':groups,'paired':paired}
    out=root/'evaluation'
    out.mkdir(exist_ok=True)
    def json_safe(value):
        if isinstance(value,float) and not math.isfinite(value): return None
        if isinstance(value,dict): return {k:json_safe(v) for k,v in value.items()}
        if isinstance(value,list): return [json_safe(v) for v in value]
        return value
    (out/'summary.json').write_text(json.dumps(json_safe(result),indent=2,allow_nan=False))
    write_summary(result,out)
    if rows:
        keys=sorted(set().union(*(r.keys() for r in rows)))
        with (out/'runs.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
        plot_results(result,out)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','groups','paired')}))
    if not args.partial:
        assert len(rows)==30 and result['audit_failures']==0, 'Missing/invalid evidence; inspect summary'


def plot_results(result,out):
    fig,axes=plt.subplots(1,3,figsize=(13,4.2))
    for ax,metric,unit in zip(axes,('ape_rmse','path_detection_pct','duration_s'),('APE RMSE (mm)','Path with recent detection (%)','Simulation time (s)')):
        for ix,scale in enumerate((4,6)):
            pair=result['paired'][str(scale)]['active-minus-baseline'][metric]
            if not pair['n']:
                continue
            factor=1000 if metric=='ape_rmse' else 1
            values=np.array(pair['differences'])*factor
            ax.scatter(np.full(len(values),ix)-.05,values,color='.4',s=18)
            mean=pair['mean']*factor
            if pair['ci95']:
                low,high=np.array(pair['ci95'])*factor
                ax.errorbar(ix+.06,mean,yerr=[[mean-low],[high-mean]],fmt='o',capsize=5,color='#2464a0')
        ax.axhline(0,color='.6',lw=1)
        labels=[f'Noise ×{scale} (n='+str(result['paired'][str(scale)]['active-minus-baseline'][metric]['n'])+')' for scale in (4,6)]
        ax.set_xticks([0,1],labels);ax.set_ylabel('Active minus baseline\n'+unit)
        ax.grid(axis='y',alpha=.2)
    fig.suptitle(('Partial series: ' if result['status']=='partial' else 'Isaac reproduction: ')+ 'paired seeds, mean and 95% t interval')
    fig.tight_layout();fig.savefig(out/'paired.png',dpi=160);plt.close(fig)


def write_summary(result,out):
    lines=['# Isaac reproduction evaluation','',
        'Original shelves_loop_a; noise ×4/×6; seeds42-46; five paired runs per mode.',
        'evo1.38.0, XY, no alignment, GT interpolation .02s, RPE1m from reference.',
        'Only goal window; no selective reruns; failed jobs retained.','',
        f"Jobs {result['evaluated_jobs']}/{result['planned_jobs']}; runner passed {result['passed_jobs']}; "
        f"Nav2 goals succeeded {int(result['navigation_goals_succeeded'])}/90; strict pose goals {result['strict_pose_goals_passed']}/90.",
        f"Data/input audit failures {result['audit_failures']}; teardown failures {result['teardown_failures']}.",'']
    def show(value,factor=1,nd=2):
        return 'unavailable' if value is None else f'{value*factor:.{nd}f}'
    for scale in ('4','6'):
        lines += ['## Noise ×'+scale,'', '| Metric | baseline | active | control |','|---|---|---|---|']
        for metric,label,factor in [('ape_rmse','APE RMSE mm',1000),('ape_max','APE max mm',1000),
            ('rpe_rmse','RPE RMSE mm',1000),('duration_s','Simulation time s',1),('path_length_m','GT path m',1),
            ('det_per_m','Detections per m',1),('path_detection_pct','Recent detection path %',1),
            ('det_coverage_pct','Recent detection time %',1),('recoveries','Reported recovery count per run',1),
            ('navigation_rtf','Navigation RTF',1)]:
            values=[]
            for mode in ('baseline','active','control'):
                s=result['groups'][scale][mode][metric]
                values.append(show(s['mean'],factor)+' ± '+show(s['std'],factor)+f" (n={s['n']})")
            lines.append('| '+label+' | '+' | '.join(values)+' |')
        lines += ['', 'Paired differences use seeds as units, mean and 95% t interval (df=4 at n=5).',
            '', '| Difference | Metric | Mean | 95% interval | n |','|---|---|---|---|---|']
        for reference,metrics in result['paired'][scale].items():
            for metric,label,factor in [('ape_rmse','APE RMSE mm',1000),('rpe_rmse','RPE RMSE mm',1000),
                ('duration_s','Simulation time s',1),('det_per_m','Detections per m',1),
                ('path_detection_pct','Recent detection path percentage points',1)]:
                p=metrics[metric];ci=p['ci95']
                interval='unavailable' if ci is None else '['+show(ci[0],factor)+', '+show(ci[1],factor)+']'
                lines.append('| '+reference+' | '+label+' | '+show(p['mean'],factor)+' | '+interval+f" | {p['n']} |")
        lines.append('')
    lines += ['## Recorded exceptions','']
    for row in result['rows']:
        if row['status']!='passed' or row.get('teardown_audit')!='passed':
            lines.append(f"- {row['batch']}: runner={row['status']}, pose={row.get('pose_acceptance')}, "
                f"teardown={row.get('teardown_audit')}, actions={row.get('goals_succeeded')}/3, "
                f"max final estimated yaw error={row.get('max_goal_estimated_yaw_error_rad')} rad.")
    lines += ['', 'All pre-seed first-TF diagnostic flags are retained. Post-initialpose errors are audited separately.',
        'Five seeds, one static map, uncontrolled MPPI RNG; intervals are descriptive, not a universal-method claim.','']
    (out/'summary.md').write_text('\n'.join(lines))


if __name__=='__main__': main()
