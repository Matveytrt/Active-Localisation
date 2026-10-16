import math
import os
import random

import pytest

from al_eval.tum import (TumWriter, apply_se2_offset, apply_se2_offset_3d,
                         format_stamp, quat_multiply, quat_to_yaw, read_tum,
                         wrap_angle, yaw_to_quat)


def _ang_close(a, b, tol=1e-12):
    return abs(wrap_angle(a - b)) < tol


# ---------------------------------------------------------------- yaw <-> q
@pytest.mark.parametrize('yaw', [0.0, 0.3, -0.3, 1.5, math.pi / 2,
                                 -math.pi / 2, 3.0, -3.0, math.pi - 1e-9])
def test_yaw_quat_roundtrip(yaw):
    q = yaw_to_quat(yaw)
    assert abs(sum(c * c for c in q) - 1.0) < 1e-12
    assert q[0] == 0.0 and q[1] == 0.0
    assert _ang_close(quat_to_yaw(*q), yaw)


def test_yaw_quat_known_values():
    qx, qy, qz, qw = yaw_to_quat(math.pi / 2)
    assert qz == pytest.approx(math.sqrt(0.5))
    assert qw == pytest.approx(math.sqrt(0.5))
    assert quat_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_quat_multiply_composes_yaw():
    q = quat_multiply(yaw_to_quat(0.4), yaw_to_quat(0.5))
    assert _ang_close(quat_to_yaw(*q), 0.9)


# ---------------------------------------------------------------- SE(2)
def test_offset_identity():
    assert apply_se2_offset(1.0, 2.0, 0.5, 0, 0, 0) == pytest.approx(
        (1.0, 2.0, 0.5))


def test_offset_translation_only():
    # spawn at (-2.0, -0.5), yaw 0: GT relative to spawn -> map/world
    x, y, yaw = apply_se2_offset(1.0, 0.0, 0.2, -2.0, -0.5, 0.0)
    assert (x, y) == pytest.approx((-1.0, -0.5))
    assert yaw == pytest.approx(0.2)


def test_offset_rotation():
    # offset rotated by 90 deg: local +x becomes +y
    x, y, yaw = apply_se2_offset(1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert yaw == pytest.approx(math.pi / 2)


def test_offset_is_left_composition_and_invertible():
    rnd = random.Random(0)
    for _ in range(200):
        p = (rnd.uniform(-5, 5), rnd.uniform(-5, 5), rnd.uniform(-3, 3))
        o = (rnd.uniform(-5, 5), rnd.uniform(-5, 5), rnd.uniform(-3, 3))
        q = apply_se2_offset(*p, *o)
        # inverse of offset: (-R^T t, -yaw)
        c, s = math.cos(o[2]), math.sin(o[2])
        inv = (-(c * o[0] + s * o[1]), -(-s * o[0] + c * o[1]), -o[2])
        back = apply_se2_offset(*q, *inv)
        assert back[0] == pytest.approx(p[0], abs=1e-9)
        assert back[1] == pytest.approx(p[1], abs=1e-9)
        assert _ang_close(back[2], p[2], 1e-9)


def test_offset_yaw_wraps():
    _, _, yaw = apply_se2_offset(0, 0, 3.0, 0, 0, 1.0)
    assert -math.pi <= yaw < math.pi
    assert _ang_close(yaw, 4.0)


def test_offset_3d_matches_2d_and_keeps_z():
    p2 = apply_se2_offset(0.7, -0.2, 0.9, 1.0, 2.0, -0.6)
    x, y, z, q = apply_se2_offset_3d(0.7, -0.2, 0.16, yaw_to_quat(0.9),
                                     1.0, 2.0, -0.6)
    assert (x, y) == pytest.approx(p2[:2])
    assert z == 0.16
    assert _ang_close(quat_to_yaw(*q), p2[2])


# ---------------------------------------------------------------- I/O
def test_format_stamp_exact():
    assert format_stamp(12, 5) == '12.000000005'
    assert format_stamp(1700000000, 123456789) == '1700000000.123456789'
    with pytest.raises(ValueError):
        format_stamp(1, 1_000_000_000)


def test_write_read_roundtrip(tmp_path):
    path = str(tmp_path / 'sub' / 'traj.tum')  # parent dir is created
    rows = []
    rnd = random.Random(1)
    t = 100.0
    for _ in range(50):
        t += 0.1
        yaw = rnd.uniform(-3, 3)
        rows.append((t, rnd.uniform(-5, 5), rnd.uniform(-5, 5), 0.16,
                     *yaw_to_quat(yaw)))
    with TumWriter(path) as w:
        for r in rows:
            w.write(*r)
        w.write_stamp(200, 7, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert w.closed and w.count == 51

    back = read_tum(path)
    assert len(back) == 51
    for a, b in zip(rows, back):
        assert b == pytest.approx(a, abs=1e-9)
    assert back[-1][0] == pytest.approx(200.000000007, abs=1e-9)

    # 9 decimals in the file
    with open(path) as f:
        lines = [ln for ln in f if not ln.startswith('#')]
    assert lines[0].split()[0] == '%.9f' % rows[0][0]
    assert len(lines[0].split()[0].split('.')[1]) == 9


def test_flushed_before_close(tmp_path):
    path = str(tmp_path / 'live.tum')
    w = TumWriter(path)
    w.write(1.0, 0, 0, 0, 0, 0, 0, 1)
    assert len(read_tum(path)) == 1  # visible without close()
    w.close()
    w.close()  # idempotent
    with pytest.raises(ValueError):
        w.write(2.0, 0, 0, 0, 0, 0, 0, 1)


def test_read_skips_truncated_tail_only(tmp_path):
    p = tmp_path / 'trunc.tum'
    p.write_text('# c\n1.0 0 0 0 0 0 0 1\n\n2.0 1 0 0 0 0 0 1\n3.0 2 0\n')
    assert len(read_tum(str(p))) == 2
    p.write_text('1.0 0 0 0 0 0 0 1\nbad line\n3.0 2 0 0 0 0 0 1\n')
    with pytest.raises(ValueError):
        read_tum(str(p))


def test_evo_can_read_if_installed(tmp_path):
    fi = pytest.importorskip('evo.tools.file_interface')
    path = str(tmp_path / 'evo.tum')
    with TumWriter(path) as w:
        for i in range(10):
            w.write(10 + 0.1 * i, 0.1 * i, 0.0, 0.0, *yaw_to_quat(0.1 * i))
    traj = fi.read_tum_trajectory_file(path)
    assert traj.num_poses == 10
    assert os.path.getsize(path) > 0
