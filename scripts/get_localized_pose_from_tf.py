# python
#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
from tf_transformations import euler_from_quaternion

class TfEchoNode(Node):
    def __init__(self, parent_frame, child_frame, rate_hz=10.0):
        super().__init__('tf_echo_py')
        self.buffer = Buffer()
        self.tl = TransformListener(self.buffer, self)
        self.parent = parent_frame
        self.child = child_frame
        self.create_timer(1.0 / rate_hz, self.timer_cb)

    def timer_cb(self):
        try:
            now = Time()
            tf = self.buffer.lookup_transform(self.parent, self.child, now)
            ts = tf.header.stamp
            secs = ts.sec + ts.nanosec * 1e-9
            t = tf.transform.translation
            q = tf.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            print(f"At time {secs:.9f}")
            print(f"Translation: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
            print(f"Rotation (quat): [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]")
            print(f"Euler (rpy): [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]")
            print()
        except Exception as e:
            # lookup may fail until transform exists; keep quiet or log debug
            self.get_logger().debug(f"lookup_transform failed: {e}")

def main():
    rclpy.init()
    parent = 'map'
    child = 'base_link'
    if len(sys.argv) >= 3:
        parent = sys.argv[1]
        child = sys.argv[2]
    node = TfEchoNode(parent, child)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()