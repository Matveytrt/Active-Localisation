"""Request a coordinated episode reset or orderly shutdown from external ROS."""
import argparse
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('reset', 'shutdown'))
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('isaac_bridge_control')
    try:
        names = ['/isaac/reset', '/isaac/reset_noise'] if args.action == 'reset' else ['/isaac/shutdown']
        for name in names:
            client = node.create_client(Trigger, name)
            if not client.wait_for_service(timeout_sec=10):
                raise TimeoutError(name+' is unavailable')
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+20
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not future.done():
                raise TimeoutError(name+' did not respond')
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            print(name+': '+response.message, flush=True)
            node.destroy_client(client)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
