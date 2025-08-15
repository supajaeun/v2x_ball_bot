#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from v2x_ball_bot_msgs.msg import BallArray

def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class ScanManager(Node):
    """
    웨이포인트를 돌며 각 지점에서 제자리 회전, /balls 토픽에서 static=True 발견 시 중단
    """
    def __init__(self):
        super().__init__("scan_manager")

        self.declare_parameter("waypoints", [])
        self.declare_parameter("spin_speed", 0.3)
        self.declare_parameter("spin_rounds", 2)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("balls_topic", "/balls")

        self.map_frame = self.get_parameter("map_frame").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.spin_speed = float(self.get_parameter("spin_speed").value)
        self.spin_rounds = int(self.get_parameter("spin_rounds").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)

        self.waypoints = []
        for w in self.get_parameter("waypoints").value:
            try:
                d = dict(w)
                self.waypoints.append({"x": float(d["x"]), "y": float(d["y"]), "yaw_deg": float(d.get("yaw_deg", 0.0))})
            except Exception as e:
                self.get_logger().warn(f"Invalid waypoint {w}: {e}")

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ball_sub = self.create_subscription(BallArray, self.get_parameter("balls_topic").value, self.on_balls, 10)

        self.ball_static_found = False
        self.wp_idx = 0
        self.state = "IDLE"
        self.t_state = time.time()
        self.timer = self.create_timer(0.5, self.tick)

    def on_balls(self, msg: BallArray):
        for b in msg.balls:
            if b.is_static:
                self.ball_static_found = True
                self.get_logger().info(f"Static ball detected: id={b.id}")
                break

    def publish_goal(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)
        z, w = yaw_to_quat(yaw)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        self.goal_pub.publish(goal)

    def spin_in_place(self, rounds, speed):
        duration = (2*math.pi*rounds)/max(0.05, abs(speed))
        t0 = time.time()
        tw = Twist()
        tw.angular.z = speed
        while rclpy.ok() and (time.time() - t0) < duration and not self.ball_static_found:
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def next_waypoint(self):
        if not self.waypoints:
            return None
        wp = self.waypoints[self.wp_idx % len(self.waypoints)]
        self.wp_idx += 1
        return wp

    def tick(self):
        if self.ball_static_found:
            self.get_logger().info("Scan complete: ball found.")
            self.timer.cancel()
            self.cmd_pub.publish(Twist())
            return

        if self.state == "IDLE":
            wp = self.next_waypoint()
            if wp:
                self.get_logger().info(f"Moving to waypoint #{self.wp_idx}: {wp}")
                self.publish_goal(wp["x"], wp["y"], wp["yaw_deg"])
                self.state = "SETTLE"
                self.t_state = time.time()

        elif self.state == "SETTLE":
            if (time.time() - self.t_state) >= self.settle_sec:
                self.state = "SPIN"

        elif self.state == "SPIN":
            self.spin_in_place(self.spin_rounds, self.spin_speed)
            self.state = "IDLE"

def main():
    rclpy.init()
    node = ScanManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
