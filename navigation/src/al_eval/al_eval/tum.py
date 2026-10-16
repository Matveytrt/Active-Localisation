"""TUM trajectory I/O and planar (SE(2)) pose helpers.

Pure Python, no ROS imports: this module is unit-tested without ROS and is
reused by the recorder node.

TUM line format (one pose per line, space separated)::

    timestamp x y z qx qy qz qw

``timestamp`` is seconds as a float written with 9 decimals; the quaternion
is Hamilton, scalar last (the same convention as geometry_msgs and evo).
"""

from __future__ import annotations

import math
import os
from typing import IO, List, Optional, Sequence, Tuple

Quat = Tuple[float, float, float, float]  # (qx, qy, qz, qw)
TumRow = Tuple[float, float, float, float, float, float, float, float]

TUM_HEADER = '# timestamp x y z qx qy qz qw'


# --------------------------------------------------------------------------
# angles / quaternions
# --------------------------------------------------------------------------
def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float) -> Quat:
    """Quaternion (qx, qy, qz, qw) of a pure rotation ``yaw`` about +z."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rotation about +z, ZYX convention) of a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product ``a * b`` for (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q))
    if n == 0.0:
        raise ValueError('zero-norm quaternion')
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


# --------------------------------------------------------------------------
# SE(2) offset:  pose' = offset (o) pose
# --------------------------------------------------------------------------
def apply_se2_offset(x: float, y: float, yaw: float,
                     ox: float, oy: float, oyaw: float
                     ) -> Tuple[float, float, float]:
    """Left-compose a planar offset with a planar pose.

    Returns ``offset (o) pose``, i.e. the pose expressed in the frame in
    which ``offset`` is the pose of the original frame::

        x'   = ox + cos(oyaw) * x - sin(oyaw) * y
        y'   = oy + sin(oyaw) * x + cos(oyaw) * y
        yaw' = wrap(oyaw + yaw)

    Typical use: ``offset`` = pose of the GT frame expressed in ``map``.
    """
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    return (ox + c * x - s * y, oy + s * x + c * y, wrap_angle(oyaw + yaw))


def apply_se2_offset_3d(x: float, y: float, z: float, q: Quat,
                        ox: float, oy: float, oyaw: float
                        ) -> Tuple[float, float, float, Quat]:
    """Same as :func:`apply_se2_offset` but for a full 3D pose.

    Position is rotated about z and shifted in x/y (z unchanged); the
    orientation is left-multiplied by the yaw offset, so roll/pitch survive.
    """
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    q_new = quat_normalize(quat_multiply(yaw_to_quat(oyaw), q))
    return (ox + c * x - s * y, oy + s * x + c * y, z, q_new)


# --------------------------------------------------------------------------
# writing / reading
# --------------------------------------------------------------------------
def format_stamp(sec: int, nanosec: int) -> str:
    """Exact decimal string of a ROS stamp (no float rounding)."""
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('nanosec out of range: %r' % nanosec)
    return '%d.%09d' % (sec, nanosec)


def format_row(stamp: str, x: float, y: float, z: float,
               qx: float, qy: float, qz: float, qw: float) -> str:
    return '%s %.9f %.9f %.9f %.9f %.9f %.9f %.9f' % (
        stamp, x, y, z, qx, qy, qz, qw)


class TumWriter:
    """Line-oriented TUM writer.

    * Creates parent directories.
    * Flushes after every line (``flush_every=1``), so a killed process
      leaves at most one partially written line, which :func:`read_tum`
      skips; ``fsync_every`` additionally forces data to disk.
    * Usable as a context manager; :meth:`close` is idempotent.
    """

    def __init__(self, path: str, header: bool = True,
                 flush_every: int = 1, fsync_every: int = 0) -> None:
        self.path = path
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        self._f: Optional[IO[str]] = open(path, 'w', encoding='ascii')
        self._flush_every = max(1, int(flush_every))
        self._fsync_every = max(0, int(fsync_every))
        self.count = 0
        if header:
            self._f.write(TUM_HEADER + '\n')
            self._f.flush()

    @property
    def closed(self) -> bool:
        return self._f is None

    def _write_line(self, line: str) -> None:
        if self._f is None:
            raise ValueError('TumWriter is closed: %s' % self.path)
        self._f.write(line + '\n')
        self.count += 1
        if self.count % self._flush_every == 0:
            self._f.flush()
        if self._fsync_every and self.count % self._fsync_every == 0:
            os.fsync(self._f.fileno())

    def write(self, t: float, x: float, y: float, z: float,
              qx: float, qy: float, qz: float, qw: float) -> None:
        """Write a pose with a float timestamp in seconds."""
        self._write_line(format_row('%.9f' % t, x, y, z, qx, qy, qz, qw))

    def write_stamp(self, sec: int, nanosec: int, x: float, y: float,
                    z: float, qx: float, qy: float, qz: float,
                    qw: float) -> None:
        """Write a pose with an exact ROS (sec, nanosec) stamp."""
        self._write_line(
            format_row(format_stamp(sec, nanosec), x, y, z, qx, qy, qz, qw))

    def close(self) -> None:
        if self._f is None:
            return
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        finally:
            self._f.close()
            self._f = None

    def __enter__(self) -> 'TumWriter':
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def write_tum(path: str, rows: Sequence[Sequence[float]]) -> None:
    """Write a whole trajectory (iterable of 8-tuples) to ``path``."""
    with TumWriter(path) as w:
        for r in rows:
            w.write(*r)


def read_tum(path: str) -> List[TumRow]:
    """Read a TUM file into a list of 8-tuples of floats.

    Blank lines and ``#`` comments are skipped. A malformed line is
    skipped only if it is the last one (truncated by a crash); otherwise
    ``ValueError`` is raised.
    """
    rows: List[TumRow] = []
    with open(path, 'r', encoding='ascii') as f:
        lines = f.read().splitlines()
    content = [(i, ln.strip()) for i, ln in enumerate(lines, 1)
               if ln.strip() and not ln.lstrip().startswith('#')]
    for k, (lineno, ln) in enumerate(content):
        parts = ln.split()
        try:
            if len(parts) != 8:
                raise ValueError('expected 8 fields, got %d' % len(parts))
            vals = tuple(float(p) for p in parts)
        except ValueError as e:
            if k == len(content) - 1:
                break  # truncated tail
            raise ValueError('%s:%d: %s' % (path, lineno, e)) from None
        rows.append(vals)  # type: ignore[arg-type]
    return rows
