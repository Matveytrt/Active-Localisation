"""Original TrajRecorder with synchronous signal-driven teardown.

Do not invalidate the rclpy context in a native signal handler while a
subscription callback is converting a message. Unexpected errors still fail.
"""
import atexit
import signal
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from al_eval.traj_recorder_node import TrajRecorder


def main():
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = TrajRecorder()
        atexit.register(node.finalize)
        executor.add_node(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.1)
    finally:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
        if node is not None:
            node.finalize()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
