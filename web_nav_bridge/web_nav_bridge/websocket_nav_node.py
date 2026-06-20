#!/usr/bin/env python3
"""
arm_bridge_node.py — ROS2 bridge node for robotic arm control panel.

Subscribes to /joint_states and forwards data to the Node.js server at
http://localhost:8080/ros-data every 100 ms (10 Hz). Reads the response and
publishes any pending command back into ROS2.
"""

import json
import threading
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from example_interfaces.msg import Float64MultiArray

# Optional: PoseCommand from custom package
try:
    from my_robot_interfaces.msg import PoseCommand
    POSE_COMMAND_AVAILABLE = True
except ImportError:
    PoseCommand = None
    POSE_COMMAND_AVAILABLE = False

SERVER_URL = "https://website-robot-arm.vercel.app/ros-data"

JOINT_NAMES_ORDERED = [
    "gripper_left_finger_joint",
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
]

NUM_JOINTS = len(JOINT_NAMES_ORDERED)


class ArmBridgeNode(Node):
    def __init__(self):
        super().__init__("arm_bridge_node")

        # Latest joint positions keyed by name
        self._joint_positions: dict[str, float] = {
            name: 0.0 for name in JOINT_NAMES_ORDERED
        }
        self._lock = threading.Lock()

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_cb,
            10,
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self._joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_command",
            10,
        )

        if POSE_COMMAND_AVAILABLE:
            self._pose_cmd_pub = self.create_publisher(
                PoseCommand,
                "/pose_command",
                10,
            )
            self.get_logger().info("PoseCommand publisher ready.")
        else:
            self._pose_cmd_pub = None
            self.get_logger().warn(
                "my_robot_interfaces not found — /pose_command publishing disabled."
            )

        # ── 10 Hz timer ────────────────────────────────────────────────────
        self._timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().info("arm_bridge_node started.")

    # ── Callback: /joint_states subscriber ─────────────────────────────────
    def _joint_states_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_positions:
                    self._joint_positions[name] = float(pos)

    # ── Callback: 10 Hz timer ──────────────────────────────────────────────
    def _timer_cb(self):
        with self._lock:
            joint_states = [
                self._joint_positions[name] for name in JOINT_NAMES_ORDERED
            ]

        payload = {
            "joint_states": joint_states,
            "joint_names": JOINT_NAMES_ORDERED,
        }

        try:
            body = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                SERVER_URL,
                data=body,
                method="POST",
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                response_data = json.loads(resp.read().decode("utf-8"))
                self.get_logger().info(f"Response data: {response_data}")
            self._handle_response(response_data)

        except urllib.error.URLError as exc:
            self.get_logger().warn(
                f"Could not reach server: {exc.reason}", throttle_duration_sec=5.0
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Unexpected error in timer callback: {exc}")

    # ── Handle server response ─────────────────────────────────────────────
    def _handle_response(self, data: dict):
        if not isinstance(data, dict):
            return

        # Joint command
        if "joint_command" in data:
            values = data["joint_command"]
            if isinstance(values, list) and len(values) == NUM_JOINTS:
                msg = Float64MultiArray()
                msg.data = [float(v) for v in values]
                self._joint_cmd_pub.publish(msg)
                self.get_logger().info(f"Published /joint_command: {msg.data}")
            else:
                self.get_logger().warn(
                    f"joint_command has wrong length: {values}"
                )

        # Pose command
        if "pose_command" in data:
            if not POSE_COMMAND_AVAILABLE or self._pose_cmd_pub is None:
                self.get_logger().warn(
                    "Received pose_command but my_robot_interfaces is unavailable."
                )
                return

            pd = data["pose_command"]
            try:
                msg = PoseCommand()
                msg.x = float(pd.get("x", 0.0))
                msg.y = float(pd.get("y", 0.0))
                msg.z = float(pd.get("z", 0.0))
                msg.roll = float(pd.get("roll", 0.0))
                msg.pitch = float(pd.get("pitch", 0.0))
                msg.yaw = float(pd.get("yaw", 0.0))
                msg.cartesian_path = bool(pd.get("cartesian_path", False))
                self._pose_cmd_pub.publish(msg)
                self.get_logger().info(f"Published /pose_command: {pd}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to build PoseCommand: {exc}")


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()