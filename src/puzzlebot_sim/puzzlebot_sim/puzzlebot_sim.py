import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Float32
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class PuzzlebotSim(Node):

    def __init__(self):
        super().__init__('puzzlebot_sim')

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

        #Robot constants
        self.r = 0.05
        self.l = 0.19

        #Puzzlebot Initial Pose
        self.v = 0.0
        self.w = 0.0
        self.theta = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        self.wr_msg = Float32()
        self.wl_msg = Float32()

        self.x_msg = Float32()
        self.y_msg = Float32()

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
        stamp = now.to_msg()

        # Kinematics
        self.theta += self.w * self.dt
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt

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

        # Wheels (cinemática diferencial)
        w_wheel = self.v / self.wheel_radius

        # TF Publish
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)

        # Publicar joints
        self.x_msg.data = self.x
        self.y_msg.data = self.y
        self.theta_pub.publish(self.theta)
        self.x_pub.publish(self.x_msg)
        self.y_pub.publish(self.y_msg)


    def define_TF(self):

        # odom -> base_footprint
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.frame_id = 'odom'
        self.base_footprint_tf.child_frame_id = 'base_footprint'

        # base_footprint -> base_link (ESTÁTICO)
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.frame_id = 'base_footprint'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.z = 0.05

        # base_link -> caster (ESTÁTICO)
        self.caster_tf = TransformStamped()
        self.caster_tf.header.frame_id = 'base_link'
        self.caster_tf.child_frame_id = 'caster_wheel'
        self.caster_tf.transform.translation.x = -0.095
        self.caster_tf.transform.translation.z = -0.03

        

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

        


def main(args=None):
    rclpy.init(args=args)

    node = PuzzlebotSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()