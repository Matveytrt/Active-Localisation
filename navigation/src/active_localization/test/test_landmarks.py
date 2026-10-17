import numpy as np
import pytest

from active_localization.landmarks import landmarks_from_dict, load_landmarks

Q_IDENT = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def _data(**kw):
    d = {'frame_id': 'map',
         'big_markers': [{'id': 1, 'position': {'x': 1.0, 'y': 2.0, 'z': 1.2},
                          'orientation': Q_IDENT}]}
    d.update(kw)
    return d


def test_load_real_file(real_landmarks_path):
    lms = load_landmarks(real_landmarks_path)
    assert sorted(m.id for m in lms) == [1, 2, 3, 4, 5]
    assert all(m.normal_xy is None for m in lms)        # normal_axis not given


def test_ids_filter(real_landmarks_path):
    assert [m.id for m in load_landmarks(real_landmarks_path, ids=[2, 4])] == [2, 4]


def test_normal_axis():
    (m,) = landmarks_from_dict(_data(), normal_axis='-x')
    assert np.allclose(m.normal_xy, [-1, 0])
    (m,) = landmarks_from_dict(_data(), normal_axis='+z')   # vertical -> unknown
    assert m.normal_xy is None


def test_bad_inputs():
    with pytest.raises(ValueError):
        landmarks_from_dict(_data(), normal_axis='w')
    with pytest.raises(ValueError):
        landmarks_from_dict(_data(frame_id='odom'))
    with pytest.raises(ValueError):
        landmarks_from_dict({'frame_id': 'map', 'big_markers': []})
