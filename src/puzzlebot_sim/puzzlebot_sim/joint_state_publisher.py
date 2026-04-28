
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('jointStatePublisher')

        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_map_to_odom', False)

        self.namespace = self.get_parameter('namespace').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publish joint_states on a namespaced topic (relative name)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.wl=0.0
        self.wr=0.0

        # Use relative topic names so they respect namespace
        self.sub_wl =self.create_subscription(
            Float32,
            'wl',
            self.wl_callback,
            10
        )

        self.sub_wr=self.create_subscription(
            Float32,
            'wr',
            self.wr_callback,
            10
        )
        #Timer
        self.timer_period=0.005#seconds
        self.timer=self.create_timer(self.timer_period,self.timer_cb)

        #Initialize the TransformBroadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        #Initialize Messages to be published
        self.msgJoints=JointState()
        # Use namespaced base_frame for header so robot_state_publisher (with frame_prefix)
        # can match the root frame. If a namespace is provided, prefix the base_frame.
        if self.namespace:
            self.msgJoints.header.frame_id = f"{self.namespace}/{self.base_frame}"
        else:
            self.msgJoints.header.frame_id = self.base_frame

        self.msgJoints.header.stamp=self.get_clock().now().to_msg()
        self.msgJoints.name = ['wheel_left_joint', 'wheel_right_joint']
        self.msgJoints.position = [0.0] * 2
        self.msgJoints.velocity = [0.0] * 2
        self.msgJoints.effort = [0.0] * 2

        self.define_TF()

    def wl_callback(self,msg):
            self.wl=msg.data
    def wr_callback(self,msg):
            self.wr=msg.data



    #Timer callback
    def timer_cb(self):
        self.msgJoints.header.stamp = self.get_clock().now().to_msg()
        self.msgJoints.position[0] += self.wl * self.timer_period
        self.msgJoints.position[1] += self.wr * self.timer_period
        self.msgJoints.velocity[0] = self.wl
        self.msgJoints.velocity[1] = self.wr
        self.publisher.publish(self.msgJoints)
            

    def define_TF(self):
                     #Transforms map to odom (with namespace support)
                        child = self.odom_frame
                        if self.namespace and not str(child).startswith(f"{self.namespace}/"):
                                child = f"{self.namespace}/{child}"
                        static_transforms=[
                            self.create_transform(
                                     parent_frame='map',
                                     child_frame=child,
                                     x=0.5, y=0.0, z=0.0,
                                     roll=0.0, pitch=0.0, yaw=0.0
                            )]
                  
                        self.tf_static_broadcaster.sendTransform(static_transforms)

    def create_transform(self,parent_frame,child_frame,
                         x,y,z,roll,pitch,yaw,
                         is_dynamic=False):
        
        tf=TransformStamped()

        tf.header.stamp=self.get_clock().now().to_msg()
        tf.header.frame_id=parent_frame
        tf.child_frame_id=child_frame

        tf.transform.translation.x=x
        tf.transform.translation.y=y
        tf.transform.translation.z=z

        q=tf_transformations.quaternion_from_euler(roll,pitch,yaw)
        tf.transform.rotation.x=q[0]
        tf.transform.rotation.y=q[1]
        tf.transform.rotation.z=q[2]
        tf.transform.rotation.w=q[3]
       

        return tf
    #create transforms Message
def main(args=None):
    rclpy.init(args=args)
    node=JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
