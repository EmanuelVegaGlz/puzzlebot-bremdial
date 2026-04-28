import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.declare_parameter('robot_frame_prefix', '')
        prefix = self.get_parameter('robot_frame_prefix').value
        ns = self.get_namespace().strip('/')
        self.fp = prefix if prefix else ns

        def frame(name):
            return f'{self.fp}/{name}' if self.fp else name
        self.frame = frame

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.wl = 0.0
        self.wr = 0.0
        self.pos_left  = 0.0
        self.pos_right = 0.0

        self.sub_wl = self.create_subscription(Float32,'wl', self.wl_callback, 10)
        self.sub_wr = self.create_subscription(Float32,'wr', self.wr_callback, 10)

        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.tf_static = StaticTransformBroadcaster(self)
        self.send_static_tfs()

        self.msgJoints = JointState()
        self.msgJoints.name = [
            frame('wheel_left_joint'),
            frame('wheel_right_joint'),
        ]
        self.msgJoints.position = [0.0, 0.0]
        self.msgJoints.velocity = [0.0, 0.0]
        self.msgJoints.effort   = [0.0, 0.0]

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def timer_cb(self):
        self.pos_left  += self.wl * self.timer_period
        self.pos_right += self.wr * self.timer_period

        self.msgJoints.header.stamp = self.get_clock().now().to_msg()
        self.msgJoints.position[0]  = self.pos_left
        self.msgJoints.position[1]  = self.pos_right
        self.msgJoints.velocity[0]  = self.wl
        self.msgJoints.velocity[1]  = self.wr
        self.publisher.publish(self.msgJoints)

    def send_static_tfs(self):
        f = self.frame
        now = self.get_clock().now().to_msg()

        map_odom = TransformStamped()
        map_odom.header.stamp    = now
        map_odom.header.frame_id = 'map'
        map_odom.child_frame_id  = f('odom')
        map_odom.transform.rotation.w = 1.0

        self.tf_static.sendTransform([map_odom])


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()