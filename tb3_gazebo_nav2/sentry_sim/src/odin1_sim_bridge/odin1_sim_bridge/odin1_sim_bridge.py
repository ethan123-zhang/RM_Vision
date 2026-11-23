"""Odin1 stand-in for the sentry simulation (S1b: standard TF chain).

In /odom  nav_msgs/Odometry  frame_id=odom  child_frame_id=base_footprint
          Wheel odometry bridged from the gz odometry-publisher-system plugin.
          This is what the mecanum base actually reports on the real car.
In /gt_odom  nav_msgs/Odometry  frame_id=map  child_frame_id=base_footprint
          Gazebo ground truth (world frame == map frame for this field).
          Stand-in for what a perfect Odin1 SLAM fix would give us.
Out /odin1/odometry  nav_msgs/Odometry  frame_id=map  child=map pose of the
          Mid-360 IMU, i.e. map_T_base plus the fixed lever arm. Kept so the
          real-hardware Odin1 consumers see the same interface as before.
Out TF map -> odom          = truth(map_T_base) * inverse(odom_T_base)
Out TF odom -> base_footprint  from /odom, z forced to 0 (planar footprint)

The old map -> sentry/imu_link TF is gone on purpose. base_link already hangs
off base_footprint via robot_state_publisher, so a second map -> imu_link edge
gave the base two parents and broke the Nav2 chain. Nav2 wants exactly
map -> odom -> base_footprint -> base_link.

/livox/imu is reported in g (linear_acceleration.z ~ 0.999, not 9.8) by the
livox gz plugin, so it is deliberately NOT fed to Nav2 or to any EKF here:
multiply by 9.80665 first if it is ever used. Nav2 only gets the 2-D /scan.

S2c frame-offset injection: params map_offset_x / map_offset_y / map_offset_yaw
(metres, metres, radians, all default 0) add a rigid offset T_off between
Odin1's map frame and the field/world frame that /gt_odom lives in:

    map_T_base_odin = T_off * truth_T_base
    map_T_odom      = map_T_base_odin * inverse(odom_T_base)

i.e. p_map = T_off * p_field, so field (0, 0) shows up at (map_offset_x,
map_offset_y) in the map the navigation stack consumes. /odin1/odometry gets
the same T_off, so both outputs stay in one frame. With the three params at 0
the node is bit-identical to S1b. The sign convention matches
`ros2 run map_align_check map_align_check`, which estimates the T_off that
takes CAD/field coordinates into Odin1's map frame.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster

# base_footprint -> livox_imu_link, from urdf/sentry.urdf.xacro:
# mid360 origin (0.05, 0, 0.30) on base_link, +0.047 to livox_frame,
# imu_link offset (0.011, 0.02329, -0.04412), base_link 0.09 above base_footprint.
IMU_LEVER = (0.061, 0.02329, 0.39288)

ODOM_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


def _rotate_by_quat(q, v):
    """Rotate vector v by quaternion q (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose2d(msg):
    p = msg.pose.pose
    return (p.position.x, p.position.y,
            _quat_to_yaw((p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)))


def _apply_offset(pose, offset):
    """Compose the injected T_off with a field-frame pose: T_off * pose."""
    x, y, yaw = pose
    ox, oy, oyaw = offset
    cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
    return (cos_o * x - sin_o * y + ox,
            sin_o * x + cos_o * y + oy,
            yaw + oyaw)


def _map_T_odom(truth, odom, offset=(0.0, 0.0, 0.0)):
    """truth = field_T_base, odom = odom_T_base, both as (x, y, yaw).

    map_T_odom = (T_off * truth) * inverse(odom_T_base), evaluated in 2-D.
    offset is T_off's (x, y, yaw); all-zero reproduces the S1b behaviour.
    """
    xt, yt, yaw_t = _apply_offset(truth, offset)
    xo, yo, _ = odom
    d = yaw_t - odom[2]
    cos_d, sin_d = math.cos(d), math.sin(d)
    return (xt - cos_d * xo + sin_d * yo,
            yt - sin_d * xo - cos_d * yo,
            d)


class Odin1SimBridge(Node):

    def __init__(self):
        super().__init__('odin1_sim_bridge')

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('noise_xy', 0.0)
        self.declare_parameter('noise_yaw', 0.0)
        self.declare_parameter('publish_cloud', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('truth_topic', '/gt_odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_odom_tf', True)
        # S2c: Odin1 map frame -> field frame rigid offset, injected on purpose.
        self.declare_parameter('map_offset_x', 0.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.declare_parameter('map_offset_yaw', 0.0)

        self.rate = float(self.get_parameter('rate').value)
        self.noise_xy = float(self.get_parameter('noise_xy').value)
        self.noise_yaw = float(self.get_parameter('noise_yaw').value)
        publish_cloud = bool(self.get_parameter('publish_cloud').value)
        odom_topic = self.get_parameter('odom_topic').value
        truth_topic = self.get_parameter('truth_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.publish_odom_tf = publish_odom_tf
        self.map_offset = (float(self.get_parameter('map_offset_x').value),
                           float(self.get_parameter('map_offset_y').value),
                           float(self.get_parameter('map_offset_yaw').value))

        if self.rate <= 0.0:
            raise ValueError('rate must be positive')
        if not all(abs(v) < 1e-9 for v in self.map_offset) and not publish_odom_tf:
            self.get_logger().warn('map_offset_* is set but publish_odom_tf is '
                                   'false: nothing will move')

        self._odom = None
        self._truth = None
        self._cloud = None
        self._warned_truth = False

        self.create_subscription(Odometry, odom_topic, self._on_odom, ODOM_QOS)
        self.create_subscription(Odometry, truth_topic, self._on_truth, ODOM_QOS)
        self.pub = self.create_publisher(Odometry, '/odin1/odometry', 10)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        if publish_odom_tf:
            # odom -> base_footprint is broadcast on every /odom message (50 Hz)
            # so the 10 Hz scan transforms never extrapolate.
            self.get_logger().info('odin1_sim_bridge: odom -> %s TF on %s'
                                   % (self.base_frame, odom_topic))

        self.cloud_pub = None
        if publish_cloud:
            self.cloud_pub = self.create_publisher(PointCloud2, '/odin1/cloud', 5)
            self.create_subscription(PointCloud2, '/livox/lidar', self._on_cloud, ODOM_QOS)

        self.get_logger().info(
            'odin1_sim_bridge up: rate=%.1f Hz noise_xy=%.3f noise_yaw=%.3f cloud=%s'
            % (self.rate, self.noise_xy, self.noise_yaw, publish_cloud))
        self.get_logger().info(
            'odin1_sim_bridge: map_offset x=%.3f y=%.3f yaw=%.4f rad (%.2f deg)'
            % (self.map_offset[0], self.map_offset[1], self.map_offset[2],
               math.degrees(self.map_offset[2])))

    def _on_odom(self, msg):
        self._odom = msg
        if self.publish_odom_tf:
            x, y, yaw = _pose2d(msg)
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf.sendTransform(tf)

        if self._truth is not None:
            truth = self._noisy_truth()
            mt = _map_T_odom(truth, _pose2d(msg), self.map_offset)
            qx, qy, qz, qw = _yaw_to_quat(mt[2])
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x = mt[0]
            tf.transform.translation.y = mt[1]
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf.sendTransform(tf)
        elif not self._warned_truth:
            self._warned_truth = True
            self.get_logger().warn('no ground truth on %s yet: map -> %s TF not '
                                   'published' % ('/gt_odom', self.odom_frame))

    def _on_truth(self, msg):
        self._truth = msg

    def _on_cloud(self, msg):
        self._cloud = msg

    def _noisy_truth(self):
        x, y, yaw = _pose2d(self._truth)
        if self.noise_xy > 0.0:
            x += random.gauss(0.0, self.noise_xy)
            y += random.gauss(0.0, self.noise_xy)
        if self.noise_yaw > 0.0:
            yaw += random.gauss(0.0, self.noise_yaw)
        return (x, y, yaw)

    def _tick(self):
        if self._truth is None or self._odom is None:
            return

        stamp = self.get_clock().now()
        if stamp.nanoseconds == 0:
            return

        x, y, yaw = _apply_offset(self._noisy_truth(), self.map_offset)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        lever = _rotate_by_quat((qx, qy, qz, qw), IMU_LEVER)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = 'sentry/imu_link'
        odom.pose.pose.position.x = x + lever[0]
        odom.pose.pose.position.y = y + lever[1]
        odom.pose.pose.position.z = self._truth.pose.pose.position.z + lever[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 1e-4 + self.noise_xy ** 2
        odom.pose.covariance[7] = 1e-4 + self.noise_xy ** 2
        odom.pose.covariance[35] = 1e-4 + self.noise_yaw ** 2
        odom.twist = self._odom.twist
        self.pub.publish(odom)

        if self.cloud_pub is not None and self._cloud is not None:
            self.cloud_pub.publish(self._cloud)


def main(args=None):
    rclpy.init(args=args)
    node = Odin1SimBridge()
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