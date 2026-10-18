"""Scenario definition, run naming and run results for the scenario runner.

Pure Python, no ROS imports: unit-tested without ROS and shared by
``scenario_runner_node`` (inside the navigation container) and
``scripts/run_experiment.sh`` (on the host, via ``python3 -m al_eval.scenario``).

Scenario YAML::

    name: example
    frame_id: map
    goal_timeout_s: 120      # per-goal timeout, wall-clock seconds
    settle_s: 2.0            # wait after readiness, before the first goal
    goals:                   # x, y [m], yaw [rad] in frame_id
      - {x: 1.0, y: 0.0, yaw: 0.0}

Run directories are ``<mode>_<NN>`` (``baseline_01``, ``active_03``), the
naming used in ``eval/README.md``; ``eval/aggregate.py`` takes the run dirs
and labels explicitly, so the name is only a convention, but it must be the
same for both modes and identical seeds must map to identical indices.

Command line (used by run_experiment.sh to validate a scenario on the host)::

    python3 -m al_eval.scenario <scenario.yaml>          # prints the name
    python3 -m al_eval.scenario <scenario.yaml> --json   # normalized scenario
"""

from __future__ import annotations

import dataclasses
import datetime
import json
import math
import numbers
import os
import re
import sys
from typing import Any, Dict, List, Optional, Sequence

# control (D-2): active params with scorer "null" + loc_layer off (ENG, T-009d)
MODES = ('baseline', 'active', 'control')
DEFAULT_ALLOWED_FRAMES = ('map',)

SCENARIO_KEYS = {'name', 'frame_id', 'goal_timeout_s', 'settle_s', 'goals'}
GOAL_KEYS = {'x', 'y', 'yaw'}
DEFAULTS = {'frame_id': 'map', 'goal_timeout_s': 120.0, 'settle_s': 2.0}

# Exit codes of scenario_runner (also recorded in result.yaml / run.yaml).
EXIT_OK = 0              # all goals succeeded
EXIT_BAD_INPUT = 2       # invalid scenario / arguments
EXIT_GOAL_FAILED = 3     # at least one goal failed, was rejected or timed out
EXIT_NOT_READY = 4       # action server or TF map->base_link not available

# Terminal per-goal statuses written to result.yaml.
GOAL_STATUSES = ('succeeded', 'aborted', 'canceled', 'timeout', 'rejected',
                 'error', 'skipped')

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


class ScenarioError(ValueError):
    """Invalid scenario file or contents."""


# --------------------------------------------------------------------------
# scenario
# --------------------------------------------------------------------------
@dataclasses.dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float


@dataclasses.dataclass(frozen=True)
class Scenario:
    name: str
    frame_id: str
    goal_timeout_s: float
    settle_s: float
    goals: List[Goal]

    def to_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'frame_id': self.frame_id,
            'goal_timeout_s': self.goal_timeout_s,
            'settle_s': self.settle_s,
            'goals': [dataclasses.asdict(g) for g in self.goals],
        }


def _number(value: Any, where: str) -> float:
    # bool is an int subclass in Python: reject it explicitly.
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ScenarioError('%s must be a number, got %r' % (where, value))
    v = float(value)
    if not math.isfinite(v):
        raise ScenarioError('%s must be finite, got %r' % (where, value))
    return v


def parse_scenario(data: Any, source: str = '<scenario>',
                   allowed_frames: Sequence[str] = DEFAULT_ALLOWED_FRAMES
                   ) -> Scenario:
    """Validate a parsed YAML document and return a :class:`Scenario`.

    Raises :class:`ScenarioError` with a message naming ``source`` and the
    offending field.
    """
    def err(msg: str) -> ScenarioError:
        return ScenarioError('%s: %s' % (source, msg))

    if not isinstance(data, dict):
        raise err('top level must be a mapping, got %s'
                  % type(data).__name__)
    unknown = sorted(set(map(str, data)) - SCENARIO_KEYS)
    if unknown:
        raise err('unknown key(s) %s (allowed: %s)'
                  % (unknown, sorted(SCENARIO_KEYS)))

    name = data.get('name')
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise err("'name' must be a non-empty string of [A-Za-z0-9_.-], "
                  'got %r' % (name,))

    frame_id = data.get('frame_id', DEFAULTS['frame_id'])
    if not isinstance(frame_id, str) or frame_id not in allowed_frames:
        raise err("unknown frame_id %r (allowed: %s)"
                  % (frame_id, list(allowed_frames)))

    try:
        timeout = _number(data.get('goal_timeout_s',
                                   DEFAULTS['goal_timeout_s']),
                          "'goal_timeout_s'")
        settle = _number(data.get('settle_s', DEFAULTS['settle_s']),
                         "'settle_s'")
    except ScenarioError as e:
        raise err(str(e)) from None
    if timeout <= 0.0:
        raise err("'goal_timeout_s' must be > 0, got %r" % timeout)
    if settle < 0.0:
        raise err("'settle_s' must be >= 0, got %r" % settle)

    if 'goals' not in data:
        raise err("missing 'goals' (list of {x, y, yaw})")
    goals_raw = data['goals']
    if not isinstance(goals_raw, list) or not goals_raw:
        raise err("'goals' must be a non-empty list of {x, y, yaw}")

    goals: List[Goal] = []
    for i, g in enumerate(goals_raw):
        where = 'goals[%d]' % i
        if not isinstance(g, dict):
            raise err('%s must be a mapping {x, y, yaw}, got %r' % (where, g))
        keys = set(map(str, g))
        missing = sorted(GOAL_KEYS - keys)
        extra = sorted(keys - GOAL_KEYS)
        if missing:
            raise err('%s: missing %s' % (where, missing))
        if extra:
            raise err('%s: unknown key(s) %s' % (where, extra))
        try:
            goals.append(Goal(x=_number(g['x'], where + '.x'),
                              y=_number(g['y'], where + '.y'),
                              yaw=_number(g['yaw'], where + '.yaw')))
        except ScenarioError as e:
            raise err(str(e)) from None

    return Scenario(name=name, frame_id=frame_id, goal_timeout_s=timeout,
                    settle_s=settle, goals=goals)


def load_scenario(path: str,
                  allowed_frames: Sequence[str] = DEFAULT_ALLOWED_FRAMES
                  ) -> Scenario:
    """Read and validate a scenario YAML file."""
    import yaml  # PyYAML ships with ROS 2; imported lazily for the CLI

    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise ScenarioError('%s: cannot read: %s' % (path, e)) from None
    except yaml.YAMLError as e:
        raise ScenarioError('%s: invalid YAML: %s' % (path, e)) from None
    return parse_scenario(data, source=path, allowed_frames=allowed_frames)


# --------------------------------------------------------------------------
# run naming / seeds
# --------------------------------------------------------------------------
def run_name(mode: str, index: int) -> str:
    """Run dir name for the ``index``-th run (1-based): ``baseline_01``."""
    if mode not in MODES:
        raise ValueError('mode must be one of %s, got %r' % (MODES, mode))
    if isinstance(index, bool) or not isinstance(index, int) or index < 1:
        raise ValueError('run index must be an int >= 1, got %r' % (index,))
    return '%s_%02d' % (mode, index)


_RUN_RE = re.compile(r'^(%s)_(\d{2,})$' % '|'.join(MODES))


def parse_run_name(name: str):
    """Inverse of :func:`run_name`: ``'active_03'`` -> ``('active', 3)``."""
    m = _RUN_RE.match(name)
    if not m or int(m.group(2)) < 1:
        raise ValueError('not a run name <mode>_<NN>: %r' % (name,))
    return m.group(1), int(m.group(2))


def run_seed(seed0: int, index: int) -> int:
    """Seed of the ``index``-th run (1-based): ``seed0 + index - 1``.

    The same list is used for every mode, so baseline_k and active_k share
    the odometry-noise seed.
    """
    if index < 1:
        raise ValueError('run index must be >= 1, got %r' % (index,))
    return int(seed0) + int(index) - 1


# --------------------------------------------------------------------------
# results
# --------------------------------------------------------------------------
def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclasses.dataclass
class GoalResult:
    index: int
    x: float
    y: float
    yaw: float
    status: str = 'skipped'
    duration_wall_s: Optional[float] = None
    duration_sim_s: Optional[float] = None
    # Sim-time stamps [s] (same clock as the recorder's TUM stamps when the
    # runner uses use_sim_time) of goal sending and of the goal's end; the
    # evaluation window is [start_sim_s of the first goal, end_sim_s of the
    # last goal] (eval/crop_run.py).
    start_sim_s: Optional[float] = None
    end_sim_s: Optional[float] = None
    # Path length of the GT odometry topic while this goal was active [m].
    path_length_m: Optional[float] = None
    # Distance of the *estimated* pose (TF frame_id->base_link) to the goal
    # when the goal ended; Nav2 decides success on this estimate.
    final_xy_error_m: Optional[float] = None
    final_yaw_error_rad: Optional[float] = None
    recoveries: Optional[int] = None
    message: str = ''

    def __post_init__(self) -> None:
        if self.status not in GOAL_STATUSES:
            raise ValueError('bad goal status %r' % self.status)

    @property
    def succeeded(self) -> bool:
        return self.status == 'succeeded'

    def set_final_pose(self, x: float, y: float, yaw: float) -> None:
        self.final_xy_error_m = math.hypot(x - self.x, y - self.y)
        self.final_yaw_error_rad = abs(wrap_angle(yaw - self.yaw))


@dataclasses.dataclass
class RunResult:
    scenario: Scenario
    run_name: str = ''
    mode: str = ''
    seed: Optional[int] = None
    start_wall: str = ''
    end_wall: str = ''
    goals: List[GoalResult] = dataclasses.field(default_factory=list)
    # Set when the run could not start (readiness timeout, bad input).
    error: str = ''
    exit_code_override: Optional[int] = None

    @classmethod
    def for_scenario(cls, scenario: Scenario, **kw) -> 'RunResult':
        rr = cls(scenario=scenario, **kw)
        rr.goals = [GoalResult(index=i, x=g.x, y=g.y, yaw=g.yaw)
                    for i, g in enumerate(scenario.goals)]
        return rr

    @property
    def n_succeeded(self) -> int:
        return sum(1 for g in self.goals if g.succeeded)

    @property
    def all_succeeded(self) -> bool:
        return bool(self.goals) and all(g.succeeded for g in self.goals)

    @property
    def exit_code(self) -> int:
        if self.exit_code_override is not None:
            return self.exit_code_override
        return EXIT_OK if self.all_succeeded else EXIT_GOAL_FAILED

    def to_dict(self) -> Dict[str, Any]:
        def num(v):
            return None if v is None else round(float(v), 6)

        return {
            'run_name': self.run_name,
            'mode': self.mode,
            'seed': self.seed,
            'scenario': self.scenario.to_dict(),
            'start_wall': self.start_wall,
            'end_wall': self.end_wall,
            'summary': {
                'n_goals': len(self.goals),
                'n_succeeded': self.n_succeeded,
                'all_succeeded': self.all_succeeded,
                'total_duration_wall_s': num(sum(
                    g.duration_wall_s or 0.0 for g in self.goals)),
                'total_path_length_m': num(sum(
                    g.path_length_m or 0.0 for g in self.goals)),
                'exit_code': self.exit_code,
                'error': self.error,
            },
            'goals': [{
                'index': g.index,
                'goal': {'x': g.x, 'y': g.y, 'yaw': g.yaw},
                'status': g.status,
                'duration_wall_s': num(g.duration_wall_s),
                'duration_sim_s': num(g.duration_sim_s),
                'start_sim_s': num(g.start_sim_s),
                'end_sim_s': num(g.end_sim_s),
                'path_length_m': num(g.path_length_m),
                'final_xy_error_m': num(g.final_xy_error_m),
                'final_yaw_error_rad': num(g.final_yaw_error_rad),
                'recoveries': g.recoveries,
                'message': g.message,
            } for g in self.goals],
        }


def now_iso() -> str:
    return datetime.datetime.now().isoformat(timespec='seconds')


def write_yaml(path: str, data: Dict[str, Any]) -> None:
    """Atomically write ``data`` as YAML (JSON fallback is valid YAML)."""
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    try:
        import yaml
        text = yaml.safe_dump(data, sort_keys=False)
    except ImportError:  # pragma: no cover - PyYAML ships with ROS 2
        text = json.dumps(data, indent=2)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def write_result(path: str, result: RunResult) -> None:
    write_yaml(path, result.to_dict())


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        prog='python3 -m al_eval.scenario',
        description='Validate a scenario YAML; print its name (or --json).')
    ap.add_argument('scenario')
    ap.add_argument('--json', action='store_true',
                    help='print the normalized scenario as JSON')
    ap.add_argument('--allowed-frames', nargs='+',
                    default=list(DEFAULT_ALLOWED_FRAMES))
    a = ap.parse_args(argv)
    try:
        sc = load_scenario(a.scenario, allowed_frames=a.allowed_frames)
    except ScenarioError as e:
        print('ERROR: %s' % e, file=sys.stderr)
        return EXIT_BAD_INPUT
    if a.json:
        print(json.dumps(sc.to_dict()))
    else:
        print(sc.name)
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
