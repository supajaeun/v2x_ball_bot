#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from v2x_ball_bot_msgs.msg import BallArray

def yaw_to_quat(yaw):
    """yaw 각도를 쿼터니언으로 변환"""
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class ScanManager(Node):
    """
    웨이포인트를 돌며 각 지점에서 제자리 회전, /balls 토픽에서 static=True 발견 시 중단
    """
    def __init__(self):
        super().__init__("scan_manager")

        # 파라미터 선언 및 기본값 설정
        self.declare_parameter("waypoints", [])
        self.declare_parameter("spin_speed", 0.3)
        self.declare_parameter("spin_rounds", 2)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("balls_topic", "/balls")

        # 파라미터 값 가져오기
        self.map_frame = self.get_parameter("map_frame").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.spin_speed = float(self.get_parameter("spin_speed").value)
        self.spin_rounds = int(self.get_parameter("spin_rounds").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)

        # 웨이포인트 파싱 개선
        self.waypoints = []
        waypoints_param = self.get_parameter("waypoints").value
        self.get_logger().info(f"Loaded waypoints: {waypoints_param}")
        
        if waypoints_param:
            for i, w in enumerate(waypoints_param):
                try:
                    if isinstance(w, dict):
                        wp = {
                            "x": float(w.get("x", 0.0)),
                            "y": float(w.get("y", 0.0)), 
                            "yaw_deg": float(w.get("yaw_deg", 0.0))
                        }
                        self.waypoints.append(wp)
                        self.get_logger().info(f"Waypoint {i}: {wp}")
                    else:
                        self.get_logger().warn(f"Invalid waypoint format at index {i}: {w}")
                except Exception as e:
                    self.get_logger().error(f"Error parsing waypoint {i}: {e}")
        
        if not self.waypoints:
            self.get_logger().warn("No valid waypoints found, using default")
            # 기본 웨이포인트 설정
            self.waypoints = [
                {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
                {"x": 1.0, "y": 0.0, "yaw_deg": 90.0},
                {"x": 1.0, "y": 1.0, "yaw_deg": 180.0},
                {"x": 0.0, "y": 1.0, "yaw_deg": 270.0}
            ]

        # 퍼블리셔/구독자 설정
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ball_sub = self.create_subscription(
            BallArray, 
            self.get_parameter("balls_topic").value, 
            self.on_balls, 
            10
        )

        # 상태 변수 초기화
        self.ball_static_found = False
        self.wp_idx = 0
        self.state = "IDLE"
        self.t_state = time.time()
        self.timer = self.create_timer(0.5, self.tick)
        
        self.get_logger().info(f"ScanManager initialized with {len(self.waypoints)} waypoints")

    def on_balls(self, msg: BallArray):
        for b in msg.balls:
            if b.is_static:
                self.ball_static_found = True
                self.get_logger().info(f"Static ball detected: id={b.id}")
                break

    def publish_goal(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)
        # yaw_to_quat 함수에서 4개 값 반환
        _, _, z, w = yaw_to_quat(yaw)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published goal: x={x}, y={y}, yaw={yaw_deg}°")

    def spin_in_place(self, rounds, speed):
        duration = (2*math.pi*rounds)/max(0.05, abs(speed))
        t0 = time.time()
        tw = Twist()
        tw.angular.z = speed
        self.get_logger().info(f"Spinning {rounds} rounds at {speed} rad/s for {duration:.1f}s")
        
        while rclpy.ok() and (time.time() - t0) < duration and not self.ball_static_found:
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        
        # 정지 명령
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info("Spin complete")

    def next_waypoint(self):
        if not self.waypoints:
            return None
        wp = self.waypoints[self.wp_idx % len(self.waypoints)]
        self.wp_idx += 1
        return wp

    def tick(self):
        if self.ball_static_found:
            self.get_logger().info("Scan complete: static ball found!")
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
                self.get_logger().info("Settling complete, starting spin")

        elif self.state == "SPIN":
            self.spin_in_place(self.spin_rounds, self.spin_speed)
            self.state = "IDLE"

def main():
    rclpy.init()
    node = ScanManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ScanManager interrupted by user")
    except Exception as e:
        node.get_logger().error(f"ScanManager error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
