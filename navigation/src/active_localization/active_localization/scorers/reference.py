"""Reference scorers. They exist to test the pipeline end to end; they are
NOT the chosen method. The method scorer (GATE-1 / ADR-001, task T-009) is
added as another module in this package and registered in `__init__.py`.
"""

import numpy as np

from .base import Scorer


class NullScorer(Scorer):
    """Score 0 everywhere. With it the yaw planner reduces to "face the
    direction of travel" and the cost field is empty, i.e. the active stack
    behaves like the baseline. Used to check that our plumbing alone does
    not change the result (control run)."""
    name = 'null'

    def score(self, x, y, yaw):
        return np.zeros(np.broadcast(np.asarray(x), np.asarray(y),
                                     np.asarray(yaw)).shape)


class VisibleCountScorer(Scorer):
    """Number of landmarks in the camera FoV and range (optionally soft
    edges, optionally weighted by 1/d^2 as a crude information proxy).

    params:
      soft:      sigmoid edge width (0 = hard test)
      weighting: 'count' | 'inv_d2'
      ref_range: range [m] at which an inv_d2 weight equals 1
      saturate:  cap on the total score ("enough is enough"), 0 = no cap
    """
    name = 'visible_count'

    def __init__(self, landmarks, camera, soft=0.0, weighting='count',
                 ref_range=3.0, saturate=0.0, **params):
        super().__init__(landmarks, camera, **params)
        if weighting not in ('count', 'inv_d2'):
            raise ValueError(f'unknown weighting {weighting!r}')
        self.soft = float(soft)
        self.weighting = weighting
        self.ref_range = float(ref_range)
        self.saturate = float(saturate)

    @property
    def score_scale(self):
        return self.saturate if self.saturate > 0 else 1.0

    def score(self, x, y, yaw):
        obs = self.observe(x, y, yaw)
        v = self.visibility(obs, self.soft)
        if self.weighting == 'inv_d2':
            v = v * (self.ref_range / np.maximum(obs.rng, 1e-3)) ** 2
        s = v.sum(axis=-1)
        if self.saturate > 0:
            s = np.minimum(s, self.saturate)
        return s
