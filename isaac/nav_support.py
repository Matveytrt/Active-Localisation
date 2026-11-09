"""Run the original noise node with an explicit episode-reset service."""
import rclpy
from std_srvs.srv import Trigger
from rclpy.executors import ExternalShutdownException
from al_odom_noise.odom_noise_node import OdomNoiseNode


def main():
    rclpy.init()
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
