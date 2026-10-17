"""Route lever: a 2D "poor localizability" cost field for the global costmap.

For every grid cell we take the best score over headings (the base is omni,
so the robot can always turn to the best view) and map it to a cost:

    cost = cost_max * (1 - min(best_score / good_score, 1)) ** power

The field is published as nav_msgs/OccupancyGrid (values 0..100) and read by
an extra `nav2_costmap_2d::StaticLayer` in the global costmap, so Theta*
prefers routes that pass where localization is good (RAVPP's repulsive
potential and FIF's information potential do the same job). Occupancy is kept
well below 100: with `trinary_costmap: false` the StaticLayer scales
0..100 linearly to 0..254, and 254 would be LETHAL. [unverified: ENG checks
the StaticLayer scaling and `use_maximum` in Humble.]

`layer_cost_grid` is the spec T-009 §3 oracle for the C++ LocalizabilityLayer
(costmap units 0..252, uint8):

    cost = round(c_max * (1 - min(s* / s_ref, 1)) ** p),  s_ref = 1 - e^-1

with s* the best score over yaw in {-pi + 2 pi k / yaw_bins}. round() is
half away from zero (= std::round for cost >= 0), i.e. floor(v + 0.5).
"""

import math
from dataclasses import dataclass

import numpy as np

S_REF = 1.0 - math.exp(-1.0)
LAYER_COST_MAX = 252        # highest cost the layer may write (253+ = INSCRIBED/LETHAL/NO_INFO)


@dataclass
class GridSpec:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def cell_centers(self):
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)          # (H, W) each, row = y


@dataclass
class CostFieldParams:
    yaw_bins: int = 36
    good_score: float = 1.0      # score at which cost drops to 0
    cost_max: int = 60           # max occupancy value written (0..100); < 100 by design
    power: float = 1.0

    @classmethod
    def from_dict(cls, d):
        d = dict(d or {})
        out = cls()
        for k, v in d.items():
            if not hasattr(out, k):
                raise ValueError(f'unknown cost field param {k!r}')
            setattr(out, k, type(getattr(out, k))(v))
        return out


def best_score_grid(scorer, grid: GridSpec, yaw_bins: int) -> np.ndarray:
    X, Y = grid.cell_centers()
    yaws = -np.pi + 2.0 * np.pi * np.arange(yaw_bins) / yaw_bins
    best = np.full(X.shape, -np.inf)
    for yaw in yaws:                     # loop keeps memory at O(H*W*L)
        best = np.maximum(best, scorer.score(X, Y, yaw))
    return best


def cost_grid(scorer, grid: GridSpec, params: CostFieldParams = CostFieldParams()) -> np.ndarray:
    """Occupancy values (int8, 0..cost_max), shape (height, width), row-major
    from origin, i.e. directly usable as OccupancyGrid.data."""
    if not 0 <= params.cost_max < 100:
        raise ValueError('cost_max must be in [0, 100)')
    best = best_score_grid(scorer, grid, params.yaw_bins)
    good = max(params.good_score, 1e-9)
    deficit = 1.0 - np.clip(best / good, 0.0, 1.0)
    return np.rint(params.cost_max * deficit ** params.power).astype(np.int8)


def layer_cost_from_score(s_star, c_max=40, power=1.0, s_ref=S_REF) -> np.ndarray:
    """Spec §3 cost (uint8, 0..c_max) from best-over-yaw scores s*."""
    if not 0 <= int(c_max) <= LAYER_COST_MAX:
        raise ValueError(f'c_max must be in [0, {LAYER_COST_MAX}]')
    if s_ref <= 0 or power <= 0:
        raise ValueError('s_ref and power must be > 0')
    s_star = np.asarray(s_star, float)
    deficit = 1.0 - np.minimum(np.maximum(s_star, 0.0) / s_ref, 1.0)
    return np.floor(int(c_max) * deficit ** power + 0.5).astype(np.uint8)


def layer_cost_grid(scorer, grid: GridSpec, c_max=40, power=1.0, yaw_bins=36,
                    s_ref=S_REF) -> np.ndarray:
    """Spec §3 LocalizabilityLayer costs (uint8), shape (height, width), row = y.

    Evaluated on every cell. Note for the C++ layer: with the spec §2
    defaults, s* stays non-negligible beyond r_max + 0.5 m (cost 38 at 2.48 m,
    < 40 up to 2.80 m from the nearest marker on the room grid), so a cut-off
    that writes c_max needs >= 2.5 m to stay within +-1 of this oracle.
    """
    return layer_cost_from_score(best_score_grid(scorer, grid, yaw_bins),
                                 c_max=c_max, power=power, s_ref=s_ref)
