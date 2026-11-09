"""Run the original noise node with an explicit episode-reset service."""
import rclpy
import os
import sys
from pathlib import Path
import yaml
from experiment_config import noise_parameters
from std_srvs.srv import Trigger
from rclpy.executors import ExternalShutdownException
from al_odom_noise.odom_noise_node import OdomNoiseNode


def main():
    repo = Path(__file__).resolve().parents[1]
    seed = int(os.environ.get('AL_ISAAC_SEED', '42'))
    scale = float(os.environ.get('AL_ISAAC_NOISE_SCALE', '1'))
    config = noise_parameters(repo, seed, scale)
    run = os.environ.get('AL_ISAAC_RUN_DIR')
    output = Path(run)/'config' if run else Path('/workspace/noise-config')
    output.mkdir(parents=True, exist_ok=True)
    path = output/'noise.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init(args=[sys.argv[0], '--ros-args', '--params-file', str(path)])
    node = OdomNoiseNode()

    def reset(_request, response):
        node.model.reset()  # Existing public reset re-seeds the original RNG.
        response.success = True
        response.message = "Odometry model and RNG reset; simulation clock remains monotonic"
        return response

    node.create_service(Trigger, "/isaac/reset_noise", reset)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
