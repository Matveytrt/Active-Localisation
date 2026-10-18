"""Golden fixtures for the C++ gtests: regenerating them must reproduce the
committed files (deterministic oracle), and they must be self-consistent."""

import importlib.util
import json
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), 'tools')
FIXTURES = os.path.join(HERE, 'fixtures')
NAMES = ('scorer_cases.json', 'layer_cost_cases.json', 'yaw_dp_case.json')


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(TOOLS, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REAL_LANDMARKS = os.path.join(HERE, '..', '..', 'rtabmap_landmarks', 'config', 'landmarks.yaml')


@pytest.fixture(scope='module')
def regenerated():
    if not os.path.exists(REAL_LANDMARKS):
        pytest.skip('rtabmap_landmarks/config/landmarks.yaml not found')
    return _load_tool('make_fixtures').build_all()


def committed(name):
    with open(os.path.join(FIXTURES, name), encoding='utf-8') as f:
        return json.load(f)


def assert_same(a, b, path='$'):
    """Structural equality; floats to 1e-12 (libm differences across machines)."""
    if isinstance(a, dict):
        assert isinstance(b, dict) and set(a) == set(b), path
        for k in a:
            assert_same(a[k], b[k], f'{path}.{k}')
    elif isinstance(a, list):
        assert isinstance(b, list) and len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            assert_same(x, y, f'{path}[{i}]')
    elif isinstance(a, float) or isinstance(b, float):
        assert isinstance(a, (int, float)) and isinstance(b, (int, float)), path
        assert math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12), f'{path}: {a} != {b}'
    else:
        assert a == b, f'{path}: {a!r} != {b!r}'


@pytest.mark.parametrize('name', NAMES)
def test_regeneration_reproduces_committed(regenerated, name):
    # round-trip through JSON so tuples / numpy scalars compare like the file
    fresh = json.loads(json.dumps(regenerated[name]))
    assert_same(fresh, committed(name))


def test_sizes_and_headers():
    sc, lc, dp = (committed(n) for n in NAMES)
    assert 45 <= len(sc['cases']) <= 60 and 45 <= len(lc['cells']) <= 70
    for d in (sc, lc, dp):
        h = d['header']
        for key in ('k_fov', 'a_min', 'k_a', 'r_min', 'k_r', 'focal_px', 'marker_size',
                    'm_sat', 's_ref', 'camera', 'markers', 'occlusion'):
            assert key in h
        assert [m['id'] for m in h['markers']] == [1, 2, 3, 4, 5]
    assert lc['header']['c_max'] == 40 and dp['header']['dp']['step'] == 0.2


def test_scorer_cases_consistent():
    sc = committed('scorer_cases.json')
    for c in sc['cases']:
        n_eff = sum(m['w'] for m in c['markers'])
        assert math.isclose(n_eff, c['n_eff'], abs_tol=1e-12)
        assert math.isclose(1 - math.exp(-n_eff), c['s'], abs_tol=1e-12)
        for m in c['markers']:
            assert math.isclose(m['w'], m['g_fov'] * m['g_area'] * m['g_near'] * m['occ'],
                                abs_tol=1e-15)
    first = sc['cases'][0]
    assert first['name'].startswith('id1 straight on, camera r=1.0')
    assert abs(first['s'] - 0.63) < 0.01
    # at least one case actually exercises the occlusion rule
    assert any(m['occ'] == 0.0 and m['g_fov'] > 0.5 for c in sc['cases'] for m in c['markers'])


def test_layer_cases_cover_all_cost_classes():
    cells = committed('layer_cost_cases.json')['cells']
    costs = [c['cost'] for c in cells]
    assert 0 in costs and 40 in costs and any(0 < v < 40 for v in costs)
    s_ref = 1 - math.exp(-1)
    for c in cells:
        assert c['cost'] == math.floor(40 * (1 - min(c['s_star'] / s_ref, 1)) + 0.5)


def test_dp_case_pins_and_lengths():
    dp = committed('yaw_dp_case.json')
    n = len(dp['points'])
    assert len(dp['yaw']) == len(dp['bin_idx']) == len(dp['travel']) == n
    assert len(dp['ds']) == n - 1 and max(dp['ds']) <= 0.2 + 1e-9
    assert dp['yaw'][0] == dp['header']['start_yaw'] and dp['yaw'][-1] == dp['header']['goal_yaw']
    assert dp['points'][0] == dp['polyline'][0] and dp['points'][-1] == dp['polyline'][-1]
