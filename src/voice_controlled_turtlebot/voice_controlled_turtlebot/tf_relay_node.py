"""
tf_relay_node — Re-publica TF de BEST_EFFORT a RELIABLE.

El Create 3 publica odom->base_link con QoS BEST_EFFORT en /tf,
pero Nav2 subscribe con RELIABLE y no lo recibe. Este nodo hace de puente.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

QOS_BE = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class TfRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_node')

        self._pub = self.create_publisher(TFMessage, '/tf', 10)
        self._sub = self.create_subscription(
            TFMessage, '/tf', self._cb, QOS_BE)

        self._relay_frames = {'odom'}
        # Evitar bucle infinito: guardar timestamps ya retransmitidos
        self._seen = set()
        self._seen_max = 200
        self.get_logger().info(
            'TF relay: BEST_EFFORT -> RELIABLE para frames con odom.')

    def _cb(self, msg):
        relay = []
        for t in msg.transforms:
            if t.header.frame_id in self._relay_frames:
                key = (t.header.stamp.sec, t.header.stamp.nanosec,
                       t.header.frame_id, t.child_frame_id)
                if key in self._seen:
                    continue
                self._seen.add(key)
                if len(self._seen) > self._seen_max:
                    self._seen.clear()
                relay.append(t)
        if relay:
            out = TFMessage()
            out.transforms = relay
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    nodo = TfRelayNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
