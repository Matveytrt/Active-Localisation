"""Independent known scene poses and marker sizes guard the SDF migration."""
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from world_source import audit


def test_original_shelves_walls_and_spawn_are_composed_in_world_frame():
    data = audit()
    shelves = [c for c in data['collisions'] if c['model'].startswith('shelf_6_')]
    assert len(shelves) == 6
    assert np.allclose(np.array(shelves[0]['transform'])[:3, 3], [-2.165, 1.2, 1.])
    assert shelves[0]['size'] == [1.32, .6, 2.]
    assert data['spawn_xyz'] == [-2., -.5, 0.]
    walls = [c for c in data['collisions'] if c['model'] == 'walls_astribot']
    assert len(walls) == 4
    wall = next(w for w in walls if w['link'] == 'Wall_11')
    matrix = np.array(wall['transform'])
    assert np.allclose(matrix[:3, 3], [-.771121+4.175, 1.67751, 1.25])
    assert abs(math.atan2(matrix[1, 0], matrix[0, 0])+math.pi/2) < 1e-5


def test_marker_ids_textures_and_physical_black_square_size():
    data = audit()
    hd17 = [m for m in data['markers'] if m['family'] == 'HD17']
    hd15 = [m for m in data['markers'] if m['family'] == 'HD15']
    assert sorted(m['id'] for m in hd17) == list(range(6))
    assert sorted(m['id'] for m in hd15) == list(range(18))
    assert np.allclose(hd17[0]['detected_edge_m'], [.08, .08])
    # Full panel .08 m includes a 10% quiet margin on either side.
    # Existing Gazebo detector config .06 differs; Isaac must use measured .064.
    assert np.allclose(hd15[0]['detected_edge_m'], [.064, .064])
    assert np.allclose(np.array(hd17[0]['transform'])[:3, 3], [-1.5, .899, 1.3915], atol=1e-7)
    assert len(data['assets']) == 25  # one shelf DAE + 24 actual marker textures
