import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from std_msgs.msg import Float32
import transforms3d
import numpy as np


class PuzzlebotSim(Node):

    def __init__(self):
        super().__init__('puzzlebot_sim')
        self.get_logger().info("PuzzlebotSim node started")

        # Parameters
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('robot_frame_prefix', '')

        x0 = self.get_parameter('x0').value
        y0 = self.get_parameter('y0').value
        theta0 = self.get_parameter('theta0').value
        prefix = self.get_parameter('robot_frame_prefix').value

        ns = self.get_namespace().strip('/')
        self.fp = prefix if prefix else ns  # frame prefix

        def frame(name):
            return f'{self.fp}/{name}' if self.fp else name
        self.frame = frame

        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel',self.cmd_vel_callback, 10)
        self.wr_pub = self.create_publisher(Float32,'wr',10)
        self.wl_pub = self.create_publisher(Float32,'wl',10)
        self.x_pub = self.create_publisher(Float32, 'sim_x',10)
        self.y_pub = self.create_publisher(Float32,'sim_y',10)
        self.theta_pub = self.create_publisher(Float32,'sim_theta', 10)
        self.pose_sim_pub = self.create_publisher(PoseStamped,'pose_sim',  10)

        # Robot constants
        self.r = 0.05
        self.l = 0.19

        # Initial pose
        self.v     = 0.0
        self.w     = 0.0
        self.x     = x0
        self.y     = y0
        self.theta = theta0

        self.last_time = self.get_clock().now()
        self.wr_msg = Float32()
        self.wl_msg = Float32()
        self.x_msg = Float32()
        self.y_msg = Float32()
        self.theta_msg = Float32()

        self.dt = 0.01

        # TF
        self.define_TF()
        self.tf_br     = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.send_static_tfs()

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        now   = self.get_clock().now()
        dt    = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        stamp = now.to_msg()

        # Kinematics
        self.theta += self.w * dt
        self.x     += self.v * np.cos(self.theta) * dt
        self.y     += self.v * np.sin(self.theta) * dt

        q = transforms3d.euler.euler2quat(0, 0, self.theta)

        # TF odom to base_footprint
        self.base_footprint_tf.header.stamp = stamp
        self.base_footprint_tf.transform.translation.x = self.x
        self.base_footprint_tf.transform.translation.y = self.y
        self.base_footprint_tf.transform.translation.z = 0.0
        self.base_footprint_tf.transform.rotation.w = q[0]
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.tf_br.sendTransform(self.base_footprint_tf)

        # Publish pose
        self.x_msg.data     = self.x
        self.y_msg.data     = self.y
        self.theta_msg.data = self.theta
        self.x_pub.publish(self.x_msg)
        self.y_pub.publish(self.y_msg)
        self.theta_pub.publish(self.theta_msg)

        pose = PoseStamped()
        pose.header.stamp    = stamp
        pose.header.frame_id = self.frame('odom')
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = q[0]
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]
        self.pose_sim_pub.publish(pose)

    def define_TF(self):
        f = self.frame

        # odom to base_footprint 
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.frame_id = f('odom')
        self.base_footprint_tf.child_frame_id  = f('base_footprint')

        # base_footprint to base_link 
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.frame_id = f('base_footprint')
        self.base_link_tf.child_frame_id  = f('base_link')
        self.base_link_tf.transform.translation.z = 0.05
        self.base_link_tf.transform.rotation.w = 1.0

        # base_link to caster_wheel 
        self.caster_tf = TransformStamped()
        self.caster_tf.header.frame_id = f('base_link')
        self.caster_tf.child_frame_id  = f('caster_wheel')
        self.caster_tf.transform.translation.x = -0.095
        self.caster_tf.transform.translation.z = -0.03
        self.caster_tf.transform.rotation.w = 1.0

    def send_static_tfs(self):
        now = self.get_clock().now().to_msg()
        self.base_link_tf.header.stamp = now
        self.caster_tf.header.stamp    = now
        self.tf_static.sendTransform([self.base_link_tf, self.caster_tf])

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.wr_msg.data = (self.v + (self.l / 2) * self.w) / self.r
        self.wl_msg.data = (self.v - (self.l / 2) * self.w) / self.r
        self.wr_pub.publish(self.wr_msg)
        self.wl_pub.publish(self.wl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSim()
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