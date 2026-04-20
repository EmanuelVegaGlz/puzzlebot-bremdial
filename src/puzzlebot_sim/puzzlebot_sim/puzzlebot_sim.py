import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class PuzzlebotSim(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #Drone Initial Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.dt = 0.01
        
        # Params
        self.v = 0.3
        self.w = 0.2
        self.wheel_radius = 0.05
        self.l_distance = 0.191

        #Define Transformations
        self.define_TF()

        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()

        self.ctrlJoints.name = [
            "wheel_left_joint",
            "wheel_right_joint",
            "caster_wheel_joint"
        ]

        self.ctrlJoints.position = [0.0] * 3
        self.ctrlJoints.velocity = [0.0] * 3
        self.ctrlJoints.effort = [0.0] * 3

        #Create Transform Boradcasters
        self.tf_br_base_footprint = TransformBroadcaster(self)

        self.tf_static = StaticTransformBroadcaster(self) # estático
        self.send_static_tfs()

        #Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

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

        # JointState
        self.ctrlJoints.header.stamp = stamp
        self.ctrlJoints.header.frame_id = "base_link" 

        # Integración de posición angular
        self.ctrlJoints.position[0] += w_wheel * self.dt   # wheel_left
        self.ctrlJoints.position[1] += w_wheel * self.dt   # wheel_right
        self.ctrlJoints.position[2] = 0.0                  # caster (fijo)

        # Opcional (pero recomendado)
        self.ctrlJoints.velocity[0] = w_wheel
        self.ctrlJoints.velocity[1] = w_wheel
        self.ctrlJoints.velocity[2] = 0.0

        # TF Publish
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)

        # Publicar joints
        self.publisher.publish(self.ctrlJoints)

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