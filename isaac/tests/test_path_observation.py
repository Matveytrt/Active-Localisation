"""Distance coverage must not reward standing still while seeing a marker."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluate_series import path_observation


def test_constant_speed_and_observation_gap():
    gt=np.array([[0,0,0],[1,1,0],[2,2,0],[3,3,0],[4,4,0]],float)
    coverage,gap=path_observation(gt,[1.])
    assert abs(coverage-25.)<1e-12
    assert abs(gap-2.)<1e-12


def test_observations_during_standing_are_not_path_coverage():
    gt=np.array([[0,0,0],[1,1,0],[2,1,0]],float)
    coverage,gap=path_observation(gt,[1.])
    assert coverage==0. and gap==1.


def test_overlapping_observations_count_distance_once():
    gt=np.array([[0,0,0],[1,1,0],[2,2,0],[3,3,0],[4,4,0]],float)
    coverage,gap=path_observation(gt,[1.,1.5])
    assert abs(coverage-37.5)<1e-12
    assert abs(gap-1.5)<1e-12
