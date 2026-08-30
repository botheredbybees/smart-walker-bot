"""walker_companion_app's ROS2 node: subscribes to pose/map/nav-status/
conversation topics, updates SharedState, and runs a stdlib HTTP server
serving the dashboard. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md for the
full design.
"""
import os
import threading
from http.server import ThreadingHTTPServer

import rclpy
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_companion_app.conversation_log import ConversationLog
from walker_companion_app.http_handler import make_handler_class
from walker_companion_app.nav_status import status_code_to_label
from walker_companion_app.occupancy_grid_json import grid_to_json
from walker_companion_app.pose_json import pose_to_json
from walker_companion_app.shared_state import SharedState


class DashboardAppNode(Node):
    def __init__(self):
        super().__init__('walker_companion_app')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('conversation_log_path', '~/.walker_companion_app/conversation.jsonl')
        self.declare_parameter('conversation_buffer_size', 50)

        http_port = self.get_parameter('http_port').value
        log_path = os.path.expanduser(self.get_parameter('conversation_log_path').value)
        buffer_size = self.get_parameter('conversation_buffer_size').value

        conversation_log = ConversationLog(log_path, buffer_size)
        self._state = SharedState(conversation_log)

        index_html_path = os.path.join(
            get_package_share_directory('walker_companion_app'), 'web', 'index.html'
        )
        with open(index_html_path, 'r') as f:
            index_html = f.read()

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._on_nav_status, 10
        )
        self.create_subscription(String, '/llm_bridge/text_in', self._on_text_in, 10)
        self.create_subscription(String, '/llm_bridge/text_out', self._on_text_out, 10)

        handler_class = make_handler_class(self._state, index_html)
        # 0.0.0.0, not just localhost - README Sec 5.5 wants this reachable
        # from a phone on the home network, not just this workstation.
        try:
            self._http_server = ThreadingHTTPServer(('0.0.0.0', http_port), handler_class)
        except OSError as e:
            raise RuntimeError(
                f"Could not bind the dashboard HTTP server to port {http_port}: {e}. "
                f"If the port is already in use, pass a different one via the launch file's "
                f"http_port argument (e.g. http_port:=8081)."
            ) from e
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

    def _on_odom(self, msg):
        orientation = msg.pose.pose.orientation
        pose = pose_to_json(msg.pose.pose.position.x, msg.pose.pose.position.y, orientation.z, orientation.w)
        self._state.set_pose(pose)

    def _on_map(self, msg):
        grid = grid_to_json(
            msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y, msg.data,
        )
        self._state.set_map(grid)

    def _on_nav_status(self, msg):
        codes = [status.status for status in msg.status_list]
        self._state.set_nav_status(status_code_to_label(codes))

    def _on_text_in(self, msg):
        self._state.add_conversation_entry('user', msg.data, self.get_clock().now().nanoseconds / 1e9)

    def _on_text_out(self, msg):
        self._state.add_conversation_entry('assistant', msg.data, self.get_clock().now().nanoseconds / 1e9)

    def stop(self):
        self._http_server.shutdown()
        self._http_thread.join(timeout=5.0)
        self._http_server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardAppNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
