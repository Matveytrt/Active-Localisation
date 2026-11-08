"""Run in external ROS Humble: require three increasing simulator timestamps."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node("al_isaac_clock_probe")
values = []
node.create_subscription(Clock, "/clock", lambda msg: values.append(
    msg.clock.sec + msg.clock.nanosec * 1e-9), qos_profile_sensor_data)
deadline = time.monotonic() + 180
while len(values) < 3 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
if len(values) < 3 or not all(b > a for a, b in zip(values, values[1:])):
    raise SystemExit(f"Clock verification failed: {values}")
print("AL_EXTERNAL_CLOCK_PASSED " + json.dumps(values), flush=True)
