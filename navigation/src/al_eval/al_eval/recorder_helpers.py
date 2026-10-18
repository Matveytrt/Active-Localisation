"""Pure helpers for the secondary recordings of ``traj_recorder``.

No ROS imports: unit-tested without ROS and reused by ``eval/summarize.py``
and ``eval/make_synthetic.py`` so that writer and reader share one format.

``landmarks.csv`` (one line per received ``rtabmap_msgs/LandmarkDetections``
message)::

    stamp,n_detections,ids
    123.400000000,2,1;3

* ``stamp`` - seconds with 9 decimals (sim time), see :func:`detections_stamp`
* ``n_detections`` - ``len(msg.landmarks)``
* ``ids`` - landmark ids in message order, ``;``-separated (empty if none)

``map_odom.tum`` uses the TUM writer of :mod:`al_eval.tum`; repeated
stamps are dropped with :class:`StampDeduper`.
"""

from __future__ import annotations

import os
from typing import IO, Iterable, List, Optional, Sequence, Tuple

from al_eval.tum import format_stamp

LANDMARKS_CSV_HEADER = 'stamp,n_detections,ids'

Stamp = Tuple[int, int]  # (sec, nanosec)
LandmarkRow = Tuple[float, int, Tuple[int, ...]]


def _is_zero(stamp: Optional[Stamp]) -> bool:
    return stamp is None or (int(stamp[0]) == 0 and int(stamp[1]) == 0)


def stamp_key(stamp_msg) -> Stamp:
    """``(sec, nanosec)`` of a builtin_interfaces/Time-like object."""
    return int(stamp_msg.sec), int(stamp_msg.nanosec)


def detections_stamp(header_stamp: Optional[Stamp],
                     landmark_stamps: Sequence[Stamp],
                     fallback: Stamp) -> Tuple[Stamp, str]:
    """Pick the stamp of one LandmarkDetections message.

    ``tf_to_fiducial`` fills only the per-landmark headers (stamp = newest
    TF sample of that marker) and leaves the array header empty, so:

    1. the array header stamp, if it is non-zero;
    2. else the newest non-zero per-landmark stamp;
    3. else ``fallback`` (the recorder's clock at reception).

    Returns ``(stamp, source)`` with source ``'header' | 'landmark' |
    'receipt'``.
    """
    if not _is_zero(header_stamp):
        return (int(header_stamp[0]), int(header_stamp[1])), 'header'
    nonzero = [(int(s[0]), int(s[1])) for s in landmark_stamps
               if not _is_zero(s)]
    if nonzero:
        return max(nonzero), 'landmark'
    return (int(fallback[0]), int(fallback[1])), 'receipt'


def format_landmark_row(stamp: str, ids: Iterable[int]) -> str:
    """One CSV line (without newline): ``stamp,n,id1;id2``."""
    ids = [int(i) for i in ids]
    return '%s,%d,%s' % (stamp, len(ids), ';'.join(str(i) for i in ids))


def parse_landmark_row(line: str) -> LandmarkRow:
    """Inverse of :func:`format_landmark_row` -> ``(t, n, ids)``."""
    parts = line.strip().split(',')
    if len(parts) != 3:
        raise ValueError('expected 3 fields, got %d' % len(parts))
    t = float(parts[0])
    n = int(parts[1])
    ids = tuple(int(x) for x in parts[2].split(';') if x.strip())
    if n < 0:
        raise ValueError('negative n_detections')
    if ids and len(ids) != n:
        raise ValueError('n_detections=%d but %d ids' % (n, len(ids)))
    return t, n, ids


class LandmarkCsvWriter:
    """Line-oriented writer for ``landmarks.csv`` (flush after every line).

    Same crash behaviour as :class:`al_eval.tum.TumWriter`: at most the last
    line can be truncated, and :func:`read_landmarks_csv` skips it.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._f: Optional[IO[str]] = open(path, 'w', encoding='ascii')
        self._f.write(LANDMARKS_CSV_HEADER + '\n')
        self._f.flush()
        self.count = 0
        self.detections = 0

    @property
    def closed(self) -> bool:
        return self._f is None

    def _write(self, stamp: str, ids: Sequence[int]) -> None:
        if self._f is None:
            raise ValueError('LandmarkCsvWriter is closed: %s' % self.path)
        self._f.write(format_landmark_row(stamp, ids) + '\n')
        self._f.flush()
        self.count += 1
        self.detections += len(ids)

    def write(self, t: float, ids: Sequence[int]) -> None:
        """Write a row with a float timestamp in seconds."""
        self._write('%.9f' % t, ids)

    def write_stamp(self, sec: int, nanosec: int, ids: Sequence[int]) -> None:
        """Write a row with an exact ROS ``(sec, nanosec)`` stamp."""
        self._write(format_stamp(sec, nanosec), ids)

    def close(self) -> None:
        if self._f is None:
            return
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        finally:
            self._f.close()
            self._f = None

    def __enter__(self) -> 'LandmarkCsvWriter':
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_landmarks_csv(path: str) -> List[LandmarkRow]:
    """Read ``landmarks.csv`` into ``[(t, n, ids), ...]`` (file order).

    The header and blank lines are skipped; a malformed line is tolerated
    only if it is the last one (truncated by a crash).
    """
    with open(path, 'r', encoding='ascii') as f:
        lines = [ln.strip() for ln in f.read().splitlines()]
    content = [(i, ln) for i, ln in enumerate(lines, 1)
               if ln and not ln.startswith('#') and ln != LANDMARKS_CSV_HEADER]
    rows: List[LandmarkRow] = []
    for k, (lineno, ln) in enumerate(content):
        try:
            rows.append(parse_landmark_row(ln))
        except ValueError as e:
            if k == len(content) - 1:
                break
            raise ValueError('%s:%d: %s' % (path, lineno, e)) from None
    return rows


class StampDeduper:
    """Accept a stamp only if it differs from the previously accepted one.

    Used for TF polled by a timer: when the transform did not advance the
    lookup returns the same stamp again, and duplicate stamps break the time
    association in evo.
    """

    def __init__(self) -> None:
        self.last: Optional[Stamp] = None
        self.skipped = 0

    def accept(self, stamp: Stamp) -> bool:
        key = (int(stamp[0]), int(stamp[1]))
        if key == self.last:
            self.skipped += 1
            return False
        self.last = key
        return True
