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

        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        self.declare_parameter('wheel_radius', 0.05) 
        self.declare_parameter('wheel_base', 0.19)

        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_frame', 'odom') 

        #Subscriber: cmd_vel 
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    

        #Publishers wheel speeds
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)

        #Publishers simulated pose
        self.x_pub = self.create_publisher(Float32, 'sim_x', 10)
        self.y_pub = self.create_publisher(Float32, 'sim_y', 10)
        self.theta_pub = self.create_publisher(Float32, 'sim_theta', 10)

        self.pose_sim_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)

        #Robot constants
        self.r = self.get_parameter('wheel_radius').value
        self.l = self.get_parameter('wheel_base').value

        #Puzzlebot Initial Pose
        self.v = 0.0
        self.w = 0.0

        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value

        self.namespace = self.get_parameter('namespace').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Apply namespace prefix to frame IDs if namespace is provided
        if self.namespace and not self.odom_frame.startswith(self.namespace):
            self.odom_frame = f"{self.namespace}/{self.odom_frame}"


        self.last_time = self.get_clock().now()

        self.wr_msg = Float32()
        self.wl_msg = Float32()

        self.x_msg = Float32()
        self.y_msg = Float32()
        self.theta_msg = Float32()

        self.dt = 0.01
        

        #Define Transformations
        self.define_TF()

        #Create Transform Boradcasters
        self.tf_br_base_footprint = TransformBroadcaster(self)

        self.tf_static = StaticTransformBroadcaster(self) # estático
        self.send_static_tfs()


        # Timer
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        stamp = now.to_msg()

        # Kinematics
        self.theta += self.w * dt
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt

        # odom -> base_footprint
        self.base_footprint_tf.header.stamp = stamp
        self.base_footprint_tf.transform.translation.x = self.x
        self.base_footprint_tf.transform.translation.y = self.y
        self.base_footprint_tf.transform.translation.z = 0.0

        q_robot = transforms3d.euler.euler2quat(0, 0, self.theta)
        self.base_footprint_tf.transform.rotation.w = q_robot[0]
        self.base_footprint_tf.transform.rotation.x = q_robot[1]
        self.base_footprint_tf.transform.rotation.y = q_robot[2]
        self.base_footprint_tf.transform.rotation.z = q_robot[3]


        # TF Publish
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)

        self.x_msg.data = self.x
        self.y_msg.data = self.y
        self.theta_msg.data = self.theta
        self.x_pub.publish(self.x_msg)
        self.y_pub.publish(self.y_msg)
        self.theta_pub.publish(self.theta_msg)

        # Create and publish pose_sim
        pose_sim = PoseStamped()
        pose_sim.header.stamp = stamp
        pose_sim.header.frame_id = self.odom_frame
        pose_sim.pose.position.x = self.x
        pose_sim.pose.position.y = self.y
        pose_sim.pose.position.z = 0.0
        pose_sim.pose.orientation.w = q_robot[0]
        pose_sim.pose.orientation.x = q_robot[1]
        pose_sim.pose.orientation.y = q_robot[2]
        pose_sim.pose.orientation.z = q_robot[3]
        self.pose_sim_pub.publish(pose_sim)
        # Apply namespace prefix to frame IDs
        base_footprint_id = 'base_footprint'
        base_link_id = 'base_link'
        caster_id = 'caster_wheel'
        
        if self.namespace:
            base_footprint_id = f"{self.namespace}/{base_footprint_id}"
            base_link_id = f"{self.namespace}/{base_link_id}"
            caster_id = f"{self.namespace}/{caster_id}"

        # odom -> base_footprint
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.frame_id = self.odom_frame
        self.base_footprint_tf.child_frame_id = base_footprint_id

        # base_footprint -> base_link (ESTÁTICO)
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.frame_id = base_footprint_id
        self.base_link_tf.child_frame_id = base_link_id
        self.base_link_tf.transform.translation.z = 0.05

        # base_link -> caster (ESTÁTICO)
        self.caster_tf = TransformStamped()
        self.caster_tf.header.frame_id = base_link_id
        self.caster_tf.child_frame_id = caster_id05

        # base_link -> caster (ESTÁTICO)
        self.caster_tf = TransformStamped()
        self.caster_tf.header.frame_id = 'base_link'
        self.caster_tf.child_frame_id = 'caster_wheel'
        self.caster_tf.transform.translation.x = -0.095
        self.caster_tf.transform.translation.z = -0.03

        self.base_link_tf.transform.rotation.w = 1.0
        self.caster_tf.transform.rotation.w = 1.0

        

    def send_static_tfs(self):
        now = self.get_clock().now().to_msg()

        self.base_link_tf.header.stamp = now
        self.caster_tf.header.stamp = now

        self.tf_static.sendTransform([
            self.base_link_tf,
            self.caster_tf
        ])
    
    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

        self.wr_msg.data = (self.v + (self.l/2)*self.w) / self.r
        self.wl_msg.data = (self.v - (self.l/2)*self.w) / self.r

        self.wr_pub.publish(self.wr_msg)
        self.wl_pub.publish(self.wl_msg)

        # self.get_logger().info(
        #     f"[CMD_VEL] v: {self.v:.3f}, w: {self.w:.3f}")
        # self.get_logger().info(
        #     f"[WHEELS] wr: {self.wr_msg.data:.3f}, wl: {self.wl_msg.data:.3f}")

        


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