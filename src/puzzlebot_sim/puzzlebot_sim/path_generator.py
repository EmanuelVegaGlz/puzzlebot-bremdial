'''
Path Generator Node, publishes goal points to the /goal topic.
Recieves all goal points from user defined parameters.
The path is defined in a 2D coordinate system by Pose2D messages.
The path is defined by a list of x,y coordinates.
Theta in Pose2D is set to 0.0 by default.
Recieves trigger messages on /next_goal to publish the next point.
'''

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Pose2D
from visualization_msgs.msg import Marker, MarkerArray as VisualizationMarkerArray

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        #logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO) # Set logger to INFO level
        self.get_logger().info("Logger set to INFO level")

        # load parameters
        raw = self.declare_parameter('path_points', [1.2, 0.0]).value
        self.path_frame = self.declare_parameter('path_frame', 'odom').value
        self.path_visualization_topic = self.declare_parameter(
            'path_visualization_topic',
            'path_generator/path_markers'
        ).value
        if len(raw) % 2 != 0:
            self.get_logger().fatal('path_points must have an even number of elements (x,y pairs)')
        self.points = [[raw[i], raw[i+1]] for i in range(0, len(raw), 2)]

        if not self.points:
            self.get_logger().error('No path points specified!')

        # Initialize index to -1 to wait for the first /next_goal message
        self.index = -1

        # publisher & subscriber
        self.goal_pub = self.create_publisher(Pose2D, 'goal', 10)
        path_marker_qos = QoSProfile(depth=1)
        path_marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_marker_pub = self.create_publisher(
            VisualizationMarkerArray,
            self.path_visualization_topic,
            path_marker_qos,
        )
        self.create_subscription(Empty, 'next_goal', self._next_goal_cb, 10)

        self.goal_published = False
        self.publish_timer = self.create_timer(0.1, self._try_publish_initial_goal)

        if not self.points:
            self.publish_timer.cancel()

        self.get_logger().info("Path Gen. Initialized!")
        # Removed debug logs for cleaner output
        self.publish_path_markers()

    def _next_goal_cb(self, msg):
        # Increment index only if there are more points
        if self.index + 1 >= len(self.points):
            self.get_logger().info('Reached end of path, no more points.')
            return  # Do not increment or publish
        self.index += 1
        self._publish(self.index)

    def _publish(self, idx):
        point = self.points[idx]

        msg = Pose2D()
        msg.x = point[0]
        msg.y = point[1]
        msg.theta = 0.0  # Default value for theta, update if needed

        self.goal_pub.publish(msg)
        self.publish_path_markers()

        self.get_logger().info(f'Publishing point #{idx}: {point}')

    def _try_publish_initial_goal(self):
        if self.goal_published:
            return

        if self.goal_pub.get_subscription_count() > 0:
            self.index = 0
            self._publish(0)
            self.goal_published = True
            self.publish_timer.cancel()

    def publish_path_markers(self):
        marker_array = self._delete_all_visualization_markers()

        if len(self.points) > 1:
            line = self._visualization_marker('path_segments', 0, Marker.LINE_STRIP)
            line.scale.x = 0.035
            line.color.r = 0.0
            line.color.g = 0.85
            line.color.b = 0.25
            line.color.a = 0.9
            line.points = [
                Point(x=float(point[0]), y=float(point[1]), z=0.08)
                for point in self.points
            ]
            marker_array.markers.append(line)

        for index, point in enumerate(self.points):
            marker = self._visualization_marker('path_points', index, Marker.CYLINDER)
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = 0.04
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.08
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.35
            marker.color.a = 0.9
            marker_array.markers.append(marker)

            label = self._visualization_marker(
                'path_point_labels',
                index,
                Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = float(point[0])
            label.pose.position.y = float(point[1])
            label.pose.position.z = 0.26
            label.scale.z = 0.16
            label.color.r = 0.75
            label.color.g = 1.0
            label.color.b = 0.8
            label.color.a = 1.0
            label.text = f'goal {index}'
            marker_array.markers.append(label)

        if 0 <= self.index < len(self.points):
            point = self.points[self.index]
            active = self._visualization_marker('path_current_goal', 0, Marker.SPHERE)
            active.pose.position.x = float(point[0])
            active.pose.position.y = float(point[1])
            active.pose.position.z = 0.16
            active.scale.x = 0.22
            active.scale.y = 0.22
            active.scale.z = 0.22
            active.color.r = 1.0
            active.color.g = 0.85
            active.color.b = 0.0
            active.color.a = 0.95
            marker_array.markers.append(active)

        self.path_marker_pub.publish(marker_array)

    def _delete_all_visualization_markers(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.path_frame
        marker.action = Marker.DELETEALL
        return VisualizationMarkerArray(markers=[marker])

    def _visualization_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.path_frame
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
