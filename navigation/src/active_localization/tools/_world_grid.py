"""2D occupancy + Nav2-like inflated costmap built from the world geometry.

There is no map file in the repo, so the offline prediction (T-011b) builds
the static map from the Gazebo world (FACTS 18.10):

  walls:  inner box x in [-4.87, 3.33], y in [-2.05, 5.40] (outside = occupied)
  shelves: boxes x in [-2.825, -1.505], [-1.125, 0.195], [0.575, 1.895]
           x rows y in [0.9, 1.5] and [3.4, 4.0] (0.38 m gaps between boxes)

[assumption] map ~ world in x/y (FACTS 17.10); the real global costmap comes
from the RTAB-Map grid (lidar), so wall/shelf edges there are noisier and may
be off by a cell or two; obstacle/denoise layers are ignored (static world).

Costs follow nav2_costmap_2d::InflationLayer::computeCost (Humble):

    d == 0                 -> 254 (LETHAL)
    0 < d <= r_inscribed   -> 253 (INSCRIBED)
    r_inscribed < d <= r_inflation -> floor(252 * exp(-k (d - r_inscribed)))
    d > r_inflation        -> 0

with d the Euclidean distance between cell centres and the nearest occupied
cell centre. Global costmap params (nav2_params_v2.yaml): resolution 0.05,
inflation_radius 0.5, cost_scaling_factor 3.0. [assumption] r_inscribed =
0.40 m ("robot radius", task T-011b); Nav2 derives it from the footprint
(0.65 x 0.75 m -> 0.325 m), which is a parameter here (`inscribed_radius`).

Pure numpy/scipy, no ROS.
"""

import math
import os
import sys
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(TOOLS_DIR)
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from active_localization.cost_field import GridSpec  # noqa: E402

LETHAL = 254
INSCRIBED = 253
MAX_NON_OBSTACLE = 252

# FACTS 18.10 (world file astribot_hd15_hd17_walls.world)
INNER_BOX = (-4.87, 3.33, -2.05, 5.40)            # xmin, xmax, ymin, ymax (free inside)
SHELF_X = ((-2.825, -1.505), (-1.125, 0.195), (0.575, 1.895))
SHELF_Y = ((0.9, 1.5), (3.4, 4.0))
SHELF_BOXES = tuple((x0, x1, y0, y1) for (y0, y1) in SHELF_Y for (x0, x1) in SHELF_X)


@dataclass
class WorldGridParams:
    resolution: float = 0.05        # global costmap resolution
    margin: float = 0.10            # grid extends this far beyond the inner wall box
    inscribed_radius: float = 0.40  # [assumption] robot radius (Nav2: 0.325 from footprint)
    inflation_radius: float = 0.50  # nav2_params_v2.yaml global inflation_layer
    cost_scaling_factor: float = 3.0


def world_grid_spec(inner_box=INNER_BOX, resolution=0.05, margin=0.10) -> GridSpec:
    """Grid covering the inner wall box plus `margin`, origin snapped to the resolution."""
    xmin, xmax, ymin, ymax = inner_box
    ox = math.floor((xmin - margin) / resolution) * resolution
    oy = math.floor((ymin - margin) / resolution) * resolution
    w = int(math.ceil((xmax + margin - ox) / resolution - 1e-9))
    h = int(math.ceil((ymax + margin - oy) / resolution - 1e-9))
    return GridSpec(origin_x=ox, origin_y=oy, resolution=resolution, width=w, height=h)


def rasterize(grid: GridSpec, inner_box=INNER_BOX, boxes=SHELF_BOXES) -> np.ndarray:
    """Occupied cells (bool, (H, W), row = y): centre outside the inner box or
    inside (closed) any obstacle box."""
    X, Y = grid.cell_centers()
    xmin, xmax, ymin, ymax = inner_box
    occ = (X < xmin) | (X > xmax) | (Y < ymin) | (Y > ymax)
    for x0, x1, y0, y1 in boxes:
        occ |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    return occ


def inflate(occupied: np.ndarray, resolution: float, inscribed_radius=0.40,
            inflation_radius=0.50, cost_scaling_factor=3.0) -> np.ndarray:
    """Nav2 InflationLayer costs (uint8) from an occupancy mask (see module doc)."""
    occupied = np.asarray(occupied, bool)
    if not occupied.any():
        return np.zeros(occupied.shape, dtype=np.uint8)
    d = ndimage.distance_transform_edt(~occupied) * resolution
    cost = np.zeros(occupied.shape, dtype=np.uint8)
    band = (d > inscribed_radius) & (d <= inflation_radius + 1e-9)
    cost[band] = np.floor(MAX_NON_OBSTACLE
                          * np.exp(-cost_scaling_factor * (d[band] - inscribed_radius)))
    cost[(d > 0) & (d <= inscribed_radius + 1e-9)] = INSCRIBED
    cost[occupied] = LETHAL
    return cost


def build_world_costmap(params: WorldGridParams = WorldGridParams(),
                        inner_box=INNER_BOX, boxes=SHELF_BOXES):
    """(grid, occupied mask, inflated master cost) for the experiment world."""
    grid = world_grid_spec(inner_box, params.resolution, params.margin)
    occ = rasterize(grid, inner_box, boxes)
    cost = inflate(occ, params.resolution, params.inscribed_radius,
                   params.inflation_radius, params.cost_scaling_factor)
    return grid, occ, cost


def combine_layer(master: np.ndarray, layer: np.ndarray) -> np.ndarray:
    """Spec T-009 §3: max(master, layer), cells >= INSCRIBED untouched."""
    master = np.asarray(master, np.uint8)
    layer = np.minimum(np.asarray(layer), MAX_NON_OBSTACLE).astype(np.uint8)
    out = np.maximum(master, layer)
    keep = master >= INSCRIBED
    out[keep] = master[keep]
    return out


def world_to_cell(grid: GridSpec, x, y):
    """(ix, iy) integer cell indices of world point(s)."""
    ix = np.floor((np.asarray(x, float) - grid.origin_x) / grid.resolution).astype(int)
    iy = np.floor((np.asarray(y, float) - grid.origin_y) / grid.resolution).astype(int)
    return ix, iy


def cell_to_world(grid: GridSpec, ix, iy):
    """Cell centre(s) in world coordinates."""
    return (grid.origin_x + (np.asarray(ix, float) + 0.5) * grid.resolution,
            grid.origin_y + (np.asarray(iy, float) + 0.5) * grid.resolution)
