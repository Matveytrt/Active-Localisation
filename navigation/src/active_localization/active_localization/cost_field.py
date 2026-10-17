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
"""

from dataclasses import dataclass

import numpy as np


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
