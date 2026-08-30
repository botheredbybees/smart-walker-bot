#!/usr/bin/env python3
"""Scripted end-to-end check for walker_companion_app - not a pytest
test.

Assumes walker_motor_driver, walker_nav (SLAM), walker_nav (Nav2), and
this package's own node are ALREADY launched (see this package's
README's "Running the end-to-end check" section for the exact
sequence). This script launches walker_llm_bridge's node itself, the
same FIFO-stdin way walker_llm_bridge/tools/verify_llm_bridge.py does -
see that file's docstring for why: /llm_bridge/text_in is published BY
that node, so driving a conversation through it needs real stdin, which
only `ros2 run` (not `ros2 launch`) provides - and requires the real
Ollama server to be reachable for the round-trip, same as that script.

Usage (after the four pre-launched nodes are running, per the README):

    python3 tools/verify_dashboard_app.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

HTTP_BASE = 'http://localhost:8081'


def _get_json(path):
    with urllib.request.urlopen(HTTP_BASE + path, timeout=5) as response:
        return json.loads(response.read())


class VerifyDriverNode(Node):
    def __init__(self):
        super().__init__('walker_companion_app_verify')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


def main():
    fifo_dir = tempfile.mkdtemp(prefix='walker_companion_app_verify_')
    fifo_path = os.path.join(fifo_dir, 'stdin_fifo')
    os.mkfifo(fifo_path)

    llm_bridge_process = subprocess.Popen(
        f'exec ros2 run walker_llm_bridge llm_bridge_node < {fifo_path}',
        shell=True, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    # Blocks until llm_bridge_process's shell redirection opens the FIFO's
    # read end - standard FIFO open-pairing, same as verify_llm_bridge.py.
    fifo_write = open(fifo_path, 'w')

    rclpy.init()
    node = VerifyDriverNode()

    try:
        time.sleep(3.0)  # let llm_bridge_node and the pre-launched nodes settle

        # --- Pose changes after a /cmd_vel command ---
        before = _get_json('/api/status')
        twist = Twist()
        twist.linear.x = 1.0
        node.cmd_pub.publish(twist)
        time.sleep(2.0)
        after = _get_json('/api/status')
        if not (after['pose']['x'] > before['pose']['x']):
            print(f"FAIL: /api/status pose.x did not increase ({before['pose']['x']} -> {after['pose']['x']})")
            return 1

        # --- Nav2 status transitions away from idle after a goal ---
        if not node.nav_client.wait_for_server(timeout_sec=10.0):
            print('FAIL: navigate_to_pose action server not available')
            return 1
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        send_goal_future = node.nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + 10.0
        while not send_goal_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        time.sleep(2.0)
        status = _get_json('/api/status')
        if status['nav_status'] == 'idle':
            print("FAIL: /api/status nav_status still 'idle' after sending a Nav2 goal")
            return 1
        print(f"Nav2 status after goal: {status['nav_status']!r}")

        # --- Map has real data ---
        grid = _get_json('/api/map')
        if grid['width'] == 0 or grid['height'] == 0:
            print(f"FAIL: /api/map returned an empty grid (width={grid['width']}, height={grid['height']})")
            return 1

        # --- Conversation log picks up an llm_bridge round-trip ---
        fifo_write.write('hello there\n')
        fifo_write.flush()

        deadline = time.monotonic() + 40.0
        conversation = []
        got_response = False
        while time.monotonic() < deadline:
            conversation = _get_json('/api/conversation')
            if any(e['role'] == 'assistant' for e in conversation):
                got_response = True
                break
            time.sleep(1.0)

        if not got_response:
            print('FAIL: no assistant entry appeared in /api/conversation within 40s')
            return 1

        if not any(e['role'] == 'user' and e['text'] == 'hello there' for e in conversation):
            print("FAIL: no matching user entry ('hello there') found in /api/conversation")
            return 1

        print('PASS: pose update, Nav2 status transition, live map, and conversation log all verified')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        fifo_write.close()
        try:
            os.killpg(os.getpgid(llm_bridge_process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            llm_bridge_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(llm_bridge_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            llm_bridge_process.wait()
        shutil.rmtree(fifo_dir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
