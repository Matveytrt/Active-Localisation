import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

REAL_LANDMARKS = os.path.join(HERE, '..', '..', 'rtabmap_landmarks', 'config', 'landmarks.yaml')


@pytest.fixture
def real_landmarks_path():
    if not os.path.exists(REAL_LANDMARKS):
        pytest.skip('rtabmap_landmarks/config/landmarks.yaml not found')
    return REAL_LANDMARKS
