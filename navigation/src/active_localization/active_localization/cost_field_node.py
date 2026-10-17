"""al_cost_field — route lever (integration option C in ARCH.md).

Subscribes to the map (`map_topic`, OccupancyGrid from RTAB-Map), copies its
geometry, computes `cost_field.cost_grid(scorer)` and publishes it latched on
`al/loc_cost`. The global costmap reads it with an extra StaticLayer:

    loc_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_topic: /al/loc_cost
      map_subscribe_transient_local: true
      trinary_costmap: false
      use_maximum: true

Recomputed only when the map geometry changes (landmarks are static).
[unverified] Not run in ROS yet; ENG checks the StaticLayer parameters above.
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .config import load_config
from .cost_field import GridSpec, cost_grid


class ALCostField(Node):
    def __init__(self):
        super().__init__('al_cost_field')
        self.declare_parameter('config_file', '')
        self.declare_parameter('scorer', '')
        self.declare_parameter('map_topic', '/map')
        cfg_path = self.get_parameter('config_file').value
        if not cfg_path:
            raise RuntimeError('parameter config_file is required')
        self.cfg = load_config(cfg_path, scorer_override=self.get_parameter('scorer').value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, 'al/loc_cost', latched)
        self.sub = self.create_subscription(OccupancyGrid,
                                            self.get_parameter('map_topic').value,
                                            self._on_map, latched)
        self._last_geom = None

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        geom = (info.width, info.height, round(info.resolution, 6),
                round(info.origin.position.x, 4), round(info.origin.position.y, 4))
        if geom == self._last_geom:
            return
        if abs(info.origin.orientation.z) > 1e-6:
            self.get_logger().warn('rotated map origin is not supported; ignoring rotation')
        grid = GridSpec(origin_x=info.origin.position.x, origin_y=info.origin.position.y,
                        resolution=info.resolution, width=info.width, height=info.height)
        data = cost_grid(self.cfg.scorer, grid, self.cfg.cost_field)
        out = OccupancyGrid()
        out.header = msg.header
        out.info = info
        out.data = data.reshape(-1).tolist()
        self.pub.publish(out)
        self._last_geom = geom
        self.get_logger().info(
            f'published al/loc_cost {info.width}x{info.height} '
            f'(scorer={self.cfg.scorer.name}, max={int(data.max())})')


def main(args=None):
    rclpy.init(args=args)
    node = ALCostField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
