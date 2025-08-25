#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Polygon, PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from v2x_ball_bot_msgs.msg import BallArray, Ball
from visualization_msgs.msg import Marker, MarkerArray

class CourtManager(Node):
    """
    코트 내부/외부 판정 및 정책 관리
    - 코트 폴리곤 내부 판정
    - 사람 감지 및 keepout 영역 생성
    - 속도 제한 영역 생성
    - 정책 발행 (/court_policy, /ball_targets)
    - 필터용 마스크 생성 (/keepout_mask, /speed_mask)
    """
    
    def __init__(self):
        super().__init__('court_manager')
        
        # 파라미터 선언
        self.declare_parameter('court_polygon', [
            [0.0, 0.0],
            [23.77, 0.0], 
            [23.77, 10.97],
            [0.0, 10.97]
        ])
        self.declare_parameter('in_margin_m', 0.2)
        self.declare_parameter('human_keepout_radius_m', 3.5)
        self.declare_parameter('speed_limit_radius_m', 4.0)
        self.declare_parameter('speed_limit_inside_mps', 0.2)
        self.declare_parameter('speed_limit_outside_mps', 1.0)
        self.declare_parameter('mode_when_in_court', 'OUTSIDE_ONLY')
        self.declare_parameter('eval_rate_hz', 10.0)
        
        # 파라미터 값 가져오기
        self.court_polygon = self.get_parameter('court_polygon').value
        self.in_margin = self.get_parameter('in_margin_m').value
        self.human_keepout_radius = self.get_parameter('human_keepout_radius_m').value
        self.speed_limit_radius = self.get_parameter('speed_limit_radius_m').value
        self.speed_limit_inside = self.get_parameter('speed_limit_inside_mps').value
        self.speed_limit_outside = self.get_parameter('speed_limit_outside_mps').value
        self.mode_when_in_court = self.get_parameter('mode_when_in_court').value
        self.eval_rate = self.get_parameter('eval_rate_hz').value
        
        # 코트 폴리곤을 numpy 배열로 변환
        self.court_points = np.array(self.court_polygon)
        
        # 상태 변수
        self.current_policy = "OUTSIDE_ONLY"  # 기본값
        self.humans_in_court = False
        self.last_policy_change = self.get_clock().now()
        
        # 퍼블리셔
        self.policy_pub = self.create_publisher(
            Marker, '/court_policy', 10)
        self.ball_targets_pub = self.create_publisher(
            BallArray, '/ball_targets', 10)
        self.keepout_mask_pub = self.create_publisher(
            OccupancyGrid, '/keepout_mask', 10)
        self.speed_mask_pub = self.create_publisher(
            OccupancyGrid, '/speed_mask', 10)
        self.keepout_info_pub = self.create_publisher(
            Marker, '/keepout_filter_info', 10)
        self.speed_info_pub = self.create_publisher(
            Marker, '/speed_filter_info', 10)
        
        # 구독자
        self.balls_sub = self.create_subscription(
            BallArray, '/balls', self.balls_callback, 10)
        
        # 타이머
        self.timer = self.create_timer(1.0/self.eval_rate, self.evaluate_court_policy)
        
        # 마스크 생성 타이머
        self.mask_timer = self.create_timer(1.0, self.update_masks)
        
        self.get_logger().info(f'CourtManager initialized with court polygon: {self.court_polygon}')
        
    def point_in_polygon(self, point, polygon):
        """점이 다각형 내부에 있는지 판정 (Ray casting algorithm)"""
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
            
        return inside
    
    def distance_to_polygon(self, point, polygon):
        """점에서 다각형 경계까지의 최단 거리"""
        x, y = point
        min_dist = float('inf')
        
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            
            # 선분까지의 거리 계산
            dist = self.point_to_line_distance(x, y, p1[0], p1[1], p2[0], p2[1])
            min_dist = min(min_dist, dist)
            
        return min_dist
    
    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """점에서 선분까지의 거리"""
        A = px - x1
        B = py - y1
        C = x2 - x1
        D = y2 - y1
        
        dot = A * C + B * D
        len_sq = C * C + D * D
        
        if len_sq == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        param = dot / len_sq
        
        if param < 0:
            xx, yy = x1, y1
        elif param > 1:
            xx, yy = x2, y2
        else:
            xx = x1 + param * C
            yy = y1 + param * D
            
        return math.sqrt((px - xx)**2 + (py - yy)**2)
    
    def evaluate_court_policy(self):
        """코트 정책 평가 및 업데이트"""
        # 현재 시간
        now = self.get_clock().now()
        
        # 코트 내부에 사람이 있는지 확인 (간단한 시뮬레이션)
        # 실제로는 사람 감지 노드에서 받아야 함
        humans_near_court = self.check_humans_near_court()
        
        # 정책 결정
        new_policy = "OUTSIDE_ONLY"
        if humans_near_court:
            new_policy = self.mode_when_in_court
            self.humans_in_court = True
        else:
            self.humans_in_court = False
            
        # 정책 변경 시 로그
        if new_policy != self.current_policy:
            self.get_logger().info(f'Court policy changed: {self.current_policy} -> {new_policy}')
            self.current_policy = new_policy
            self.last_policy_change = now
            
        # 정책 발행
        self.publish_court_policy()
        
    def check_humans_near_court(self):
        """코트 근처에 사람이 있는지 확인 (시뮬레이션용)"""
        # 실제로는 사람 감지 노드에서 받아야 함
        # 지금은 간단한 시뮬레이션
        import random
        return random.random() < 0.3  # 30% 확률로 사람 있음
    
    def publish_court_policy(self):
        """코트 정책을 마커로 발행"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "court_policy"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 코트 중앙에 텍스트 배치
        center_x = sum(p[0] for p in self.court_polygon) / len(self.court_polygon)
        center_y = sum(p[1] for p in self.court_polygon) / len(self.court_polygon)
        
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 1.0
        
        marker.text = f"Policy: {self.current_policy}"
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 0.5
        
        if self.current_policy == "OUTSIDE_ONLY":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.policy_pub.publish(marker)
        
    def balls_callback(self, msg):
        """공 감지 메시지 처리 및 정책 적용"""
        if not msg.balls:
            return
            
        filtered_balls = []
        
        for ball in msg.balls:
            # 코트 정책에 따른 필터링
            if self.should_include_ball(ball):
                filtered_balls.append(ball)
                
        # 필터링된 공 후보 발행
        if filtered_balls:
            filtered_msg = BallArray()
            filtered_msg.balls = filtered_balls
            self.ball_targets_pub.publish(filtered_msg)
            
    def should_include_ball(self, ball):
        """공을 포함해야 하는지 정책에 따라 판정"""
        if self.current_policy == "OUTSIDE_ONLY":
            # 코트 외부 공만 포함
            return not self.point_in_polygon([ball.x, ball.y], self.court_polygon)
        elif self.current_policy == "WAIT":
            # 모든 공 포함하지 않음
            return False
        else:
            # 기본적으로 모든 공 포함
            return True
            
    def update_masks(self):
        """keepout 및 speed 마스크 업데이트"""
        self.create_keepout_mask()
        self.create_speed_mask()
        
    def create_keepout_mask(self):
        """keepout 마스크 생성"""
        # 간단한 마스크 생성 (실제로는 더 정교하게)
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info.resolution = 0.05  # 5cm 해상도
        mask.info.width = 500  # 25m
        mask.info.height = 250  # 12.5m
        mask.info.origin.position.x = -2.0
        mask.info.origin.position.y = -2.0
        
        # 마스크 데이터 생성
        data = []
        for y in range(mask.info.height):
            for x in range(mask.info.width):
                world_x = x * mask.info.resolution + mask.info.origin.position.x
                world_y = y * mask.info.resolution + mask.info.origin.position.y
                
                # 코트 내부는 keepout (255)
                if self.point_in_polygon([world_x, world_y], self.court_polygon):
                    data.append(255)
                else:
                    data.append(0)
                    
        mask.data = data
        self.keepout_mask_pub.publish(mask)
        
        # 필터 정보 발행
        info = Marker()
        info.header.frame_id = "map"
        info.header.stamp = self.get_clock().now().to_msg()
        info.ns = "keepout_filter_info"
        info.id = 1
        info.type = Marker.TEXT_VIEW_FACING
        info.action = Marker.ADD
        info.pose.position.x = 0.0
        info.pose.position.y = 15.0
        info.pose.position.z = 1.0
        info.text = "Keepout Filter: 255=forbidden, 0=free"
        info.scale.x = 1.0
        info.scale.y = 1.0
        info.scale.z = 0.3
        info.color.r = 1.0
        info.color.g = 0.0
        info.color.b = 0.0
        info.color.a = 0.8
        self.keepout_info_pub.publish(info)
        
    def create_speed_mask(self):
        """speed 마스크 생성"""
        # 간단한 speed 마스크 생성
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info.resolution = 0.05
        mask.info.width = 500
        mask.info.height = 250
        mask.info.origin.position.x = -2.0
        mask.info.origin.position.y = -2.0
        
        # 마스크 데이터 생성
        data = []
        for y in range(mask.info.height):
            for x in range(mask.info.width):
                world_x = x * mask.info.resolution + mask.info.origin.position.x
                world_y = y * mask.info.resolution + mask.info.origin.position.y
                
                # 코트 내부는 저속 (20%), 외부는 고속 (80%)
                if self.point_in_polygon([world_x, world_y], self.court_polygon):
                    data.append(20)  # 20% 속도
                else:
                    data.append(80)  # 80% 속도
                    
        mask.data = data
        self.speed_mask_pub.publish(mask)
        
        # 필터 정보 발행
        info = Marker()
        info.header.frame_id = "map"
        info.header.stamp = self.get_clock().now().to_msg()
        info.ns = "speed_filter_info"
        info.id = 2
        info.type = Marker.TEXT_VIEW_FACING
        info.action = Marker.ADD
        info.pose.position.x = 0.0
        info.pose.position.y = 16.0
        info.pose.position.z = 1.0
        info.text = "Speed Filter: 20%=slow, 80%=fast"
        info.scale.x = 1.0
        info.scale.y = 1.0
        info.scale.z = 0.3
        info.color.r = 0.0
        info.color.g = 0.0
        info.color.b = 1.0
        info.color.a = 0.8
        self.speed_info_pub.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = CourtManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
