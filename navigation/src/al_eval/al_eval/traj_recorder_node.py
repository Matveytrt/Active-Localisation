"""traj_recorder: write GT and estimated trajectories as TUM files.

Outputs in ``<output_dir>/<run_name>/``:

* ``gt.tum``  - every message of ``gt_topic`` (nav_msgs/Odometry), stamp from
  the message header, pose with the SE(2) offset ``gt_offset_*`` applied
  (pose_map = offset o pose_gt).
* ``est.tum`` - TF ``map_frame -> base_frame`` polled at ``rate_hz``; the
  latest available transform is looked up and written with the stamp of the
  transform itself (not the timer time). Unavailable TF / repeated stamps are
  skipped and counted.
* ``landmarks.csv`` (``record_landmarks``, default true) - one line per
  ``rtabmap_msgs/LandmarkDetections`` message on ``landmarks_topic``
  (``/landmark_detections`` from ``tf_to_fiducial``): ``stamp,n_detections,
  ids`` (ids ``;``-separated), see ``al_eval.recorder_helpers``.
  ``rtabmap_msgs`` is imported lazily: if it is missing, this recording is
  disabled with a warning (reason in ``meta.yaml``).
* ``map_odom.tum`` (``record_map_odom``, default true) - TF ``map_frame ->
  odom_frame`` (the RTAB-Map correction) polled at ``rate_hz`` like est,
  stamp of the transform, repeated stamps skipped and counted.
* ``meta.yaml`` - parameters, start/stop times, counters, the first raw GT
  pose and its frame ids (to help pin down which frame /odom_gt is in).

The node is deliberately thin: all formatting lives in ``al_eval.tum`` and
``al_eval.recorder_helpers``.

[unverified] The landmarks / map->odom recording (message field access,
QoS match with tf_to_fiducial, map->odom TF rate and stamps) was written
without ROS; ENG: check on the testbed.
"""

from __future__ import annotations

import atexit
import datetime
import os
from typing import Any, Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from al_eval.recorder_helpers import (LandmarkCsvWriter, StampDeduper,
                                      detections_stamp, stamp_key)
from al_eval.tum import TumWriter, apply_se2_offset_3d, quat_to_yaw

try:  # Humble has it; keep a fallback for older rclpy
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    class ExternalShutdownException(Exception):  # type: ignore[no-redef]
        pass


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _import_landmark_msg():
    """(LandmarkDetections class, '') or (None, reason) - lazy import."""
    try:
        from rtabmap_msgs.msg import LandmarkDetections
    except ImportError as e:  # rtabmap_msgs not built / not sourced
        return None, 'cannot import rtabmap_msgs.msg.LandmarkDetections: %s' % e
    return LandmarkDetections, ''


class TrajRecorder(Node):

    def __init__(self) -> None:
        super().__init__('traj_recorder')

        self.declare_parameter('output_dir', '~/al_runs')
        self.declare_parameter('run_name', '')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gt_topic', '/odom_gt')
        self.declare_parameter('gt_offset_x', 0.0)
        self.declare_parameter('gt_offset_y', 0.0)
        self.declare_parameter('gt_offset_yaw', 0.0)
        self.declare_parameter('record_gt', True)
        self.declare_parameter('overwrite', False)
        self.declare_parameter('record_landmarks', True)
        self.declare_parameter('landmarks_topic', '/landmark_detections')
        self.declare_parameter('record_map_odom', True)
        self.declare_parameter('odom_frame', 'odom')

        gp = self.get_parameter
        self.output_dir = os.path.expanduser(gp('output_dir').value)
        run_name = gp('run_name').value or datetime.datetime.now().strftime(
            'run_%Y%m%d_%H%M%S')
        self.run_name = run_name
        self.rate_hz = float(gp('rate_hz').value)
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.gt_topic = gp('gt_topic').value
        self.gt_offset = (float(gp('gt_offset_x').value),
                          float(gp('gt_offset_y').value),
                          float(gp('gt_offset_yaw').value))
        self.record_gt = bool(gp('record_gt').value)
        overwrite = bool(gp('overwrite').value)
        self.record_landmarks = bool(gp('record_landmarks').value)
        self.landmarks_topic = gp('landmarks_topic').value
        self.record_map_odom = bool(gp('record_map_odom').value)
        self.odom_frame = gp('odom_frame').value
        if self.rate_hz <= 0.0:
            raise ValueError('rate_hz must be > 0')

        self.run_dir = os.path.join(self.output_dir, self.run_name)
        est_path = os.path.join(self.run_dir, 'est.tum')
        gt_path = os.path.join(self.run_dir, 'gt.tum')
        lm_path = os.path.join(self.run_dir, 'landmarks.csv')
        mo_path = os.path.join(self.run_dir, 'map_odom.tum')
        if not overwrite and any(os.path.exists(p) for p in (
                est_path, gt_path, lm_path, mo_path)):
            raise RuntimeError(
                'run dir already has recordings: %s (set overwrite:=true '
                'or choose another run_name)' % self.run_dir)
        os.makedirs(self.run_dir, exist_ok=True)

        self.counters: Dict[str, int] = {
            'est_written': 0,
            'est_skipped_tf_unavailable': 0,
            'est_skipped_duplicate_stamp': 0,
            'gt_written': 0,
            'map_odom_written': 0,
            'map_odom_skipped_tf_unavailable': 0,
            'map_odom_skipped_duplicate_stamp': 0,
            'landmark_msgs_written': 0,
            'landmark_detections_total': 0,
        }
        # Why a secondary recording is off ('' = on); goes to meta.yaml.
        self.disabled: Dict[str, str] = {}
        self.landmark_stamp_sources: Dict[str, int] = {}
        self.first_map_odom_error: Optional[str] = None
        self.first_tf_error: Optional[str] = None
        self.gt_info: Dict[str, Any] = {}
        self.start_wall = datetime.datetime.now().isoformat()
        self.start_ros = _stamp_sec(self.get_clock().now().to_msg())
        self._last_est_stamp = None
        self._finalized = False

        self.est_writer = TumWriter(est_path)
        self.gt_writer: Optional[TumWriter] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.record_gt:
            self.gt_writer = TumWriter(gt_path)
            # BEST_EFFORT subscriber matches both reliable and best-effort
            # publishers.
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, self.gt_topic, self._on_gt,
                                     qos)

        self.lm_writer: Optional[LandmarkCsvWriter] = None
        if self.record_landmarks:
            msg_cls, reason = _import_landmark_msg()
            if msg_cls is None:
                self.disabled['landmarks'] = reason
                self.get_logger().warn(
                    'landmark recording disabled: %s' % reason)
            else:
                self.lm_writer = LandmarkCsvWriter(lm_path)
                # tf_to_fiducial publishes RELIABLE, depth 10 (default QoS).
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                                 reliability=ReliabilityPolicy.RELIABLE)
                self.create_subscription(msg_cls, self.landmarks_topic,
                                         self._on_landmarks, qos)
        else:
            self.disabled['landmarks'] = 'record_landmarks:=false'

        self.mo_writer: Optional[TumWriter] = None
        self.mo_dedup = StampDeduper()
        if self.record_map_odom:
            self.mo_writer = TumWriter(mo_path)
        else:
            self.disabled['map_odom'] = 'record_map_odom:=false'

        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)
        self.get_logger().info(
            'recording to %s (est: TF %s->%s @ %.1f Hz; gt: %s, offset '
            'x=%.3f y=%.3f yaw=%.4f; landmarks: %s; map_odom: %s)' % (
                self.run_dir, self.map_frame, self.base_frame, self.rate_hz,
                self.gt_topic if self.record_gt else 'off', *self.gt_offset,
                self.landmarks_topic if self.lm_writer else 'off',
                '%s->%s' % (self.map_frame, self.odom_frame)
                if self.mo_writer else 'off'))

    # ------------------------------------------------------------ callbacks
    def _on_gt(self, msg: Odometry) -> None:
        if self.gt_writer is None or self.gt_writer.closed:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        if not self.gt_info:
            self.gt_info = {
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'first_stamp': _stamp_sec(msg.header.stamp),
                'first_pose_raw': {
                    'x': p.x, 'y': p.y, 'z': p.z,
                    'yaw': quat_to_yaw(o.x, o.y, o.z, o.w)},
            }
        x, y, z, q = apply_se2_offset_3d(p.x, p.y, p.z,
                                         (o.x, o.y, o.z, o.w),
                                         *self.gt_offset)
        st = msg.header.stamp
        self.gt_writer.write_stamp(st.sec, st.nanosec, x, y, z, *q)
        self.counters['gt_written'] += 1

    def _on_landmarks(self, msg) -> None:
        if self.lm_writer is None or self.lm_writer.closed:
            return
        lms = list(msg.landmarks)
        header = getattr(msg, 'header', None)  # [unverified] field exists
        stamp, source = detections_stamp(
            stamp_key(header.stamp) if header is not None else None,
            [stamp_key(ld.header.stamp) for ld in lms],
            stamp_key(self.get_clock().now().to_msg()))
        self.landmark_stamp_sources[source] = (
            self.landmark_stamp_sources.get(source, 0) + 1)
        self.lm_writer.write_stamp(stamp[0], stamp[1],
                                   [int(ld.id) for ld in lms])
        self.counters['landmark_msgs_written'] += 1
        self.counters['landmark_detections_total'] += len(lms)

    def _record_map_odom(self) -> None:
        if self.mo_writer is None or self.mo_writer.closed:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time())
        except TransformException as e:
            self.counters['map_odom_skipped_tf_unavailable'] += 1
            if self.first_map_odom_error is None:
                self.first_map_odom_error = str(e)
            return
        st = tf.header.stamp
        if not self.mo_dedup.accept(stamp_key(st)):
            self.counters['map_odom_skipped_duplicate_stamp'] += 1
            return
        t = tf.transform.translation
        r = tf.transform.rotation
        self.mo_writer.write_stamp(st.sec, st.nanosec, t.x, t.y, t.z,
                                   r.x, r.y, r.z, r.w)
        self.counters['map_odom_written'] += 1

    def _on_timer(self) -> None:
        self._record_map_odom()
        if self.est_writer.closed:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException as e:
            self.counters['est_skipped_tf_unavailable'] += 1
            if self.first_tf_error is None:
                self.first_tf_error = str(e)
            self.get_logger().warn(
                'TF %s->%s unavailable: %s' % (
                    self.map_frame, self.base_frame, e),
                throttle_duration_sec=5.0)
            return
        st = tf.header.stamp
        key = (st.sec, st.nanosec)
        if key == self._last_est_stamp:
            # TF did not advance since the last tick: same pose, skip.
            self.counters['est_skipped_duplicate_stamp'] += 1
            return
        self._last_est_stamp = key
        t = tf.transform.translation
        r = tf.transform.rotation
        self.est_writer.write_stamp(st.sec, st.nanosec, t.x, t.y, t.z,
                                    r.x, r.y, r.z, r.w)
        self.counters['est_written'] += 1

    # ------------------------------------------------------------ shutdown
    def finalize(self) -> None:
        """Close files and write meta.yaml. Safe to call more than once."""
        if self._finalized:
            return
        self._finalized = True
        for w in (self.est_writer, self.gt_writer, self.lm_writer,
                  self.mo_writer):
            if w is not None:
                try:
                    w.close()
                except Exception as e:  # noqa: BLE001
                    print('[traj_recorder] close failed: %s' % e)
        try:
            end_ros = _stamp_sec(self.get_clock().now().to_msg())
        except Exception:  # noqa: BLE001  (context may be gone)
            end_ros = None
        meta = {
            'run_name': self.run_name,
            'run_dir': self.run_dir,
            'params': {
                'output_dir': self.output_dir,
                'rate_hz': self.rate_hz,
                'map_frame': self.map_frame,
                'base_frame': self.base_frame,
                'gt_topic': self.gt_topic,
                'gt_offset_x': self.gt_offset[0],
                'gt_offset_y': self.gt_offset[1],
                'gt_offset_yaw': self.gt_offset[2],
                'record_gt': self.record_gt,
                'record_landmarks': self.record_landmarks,
                'landmarks_topic': self.landmarks_topic,
                'record_map_odom': self.record_map_odom,
                'odom_frame': self.odom_frame,
            },
            'start_wall': self.start_wall,
            'stop_wall': datetime.datetime.now().isoformat(),
            'start_ros_time': self.start_ros,
            'stop_ros_time': end_ros,
            'counters': dict(self.counters),
            'first_tf_error': self.first_tf_error,
            'gt_info': self.gt_info,
            'secondary_disabled': dict(self.disabled),
            'landmark_stamp_sources': dict(self.landmark_stamp_sources),
            'first_map_odom_error': self.first_map_odom_error,
        }
        _write_yaml(os.path.join(self.run_dir, 'meta.yaml'), meta)
        print('[traj_recorder] closed %s: %s' % (self.run_dir,
                                                 self.counters))


def _write_yaml(path: str, data: Dict[str, Any]) -> None:
    tmp = path + '.tmp'
    try:
        import yaml
        text = yaml.safe_dump(data, sort_keys=False)
    except ImportError:  # pragma: no cover - PyYAML ships with ROS 2
        import json
        text = json.dumps(data, indent=2)  # JSON is valid YAML
    with open(tmp, 'w') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrajRecorder()
        atexit.register(node.finalize)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
