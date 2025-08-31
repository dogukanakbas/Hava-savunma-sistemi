#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from nav_msgs.msg import Path
from air_defense_system.msg import HssKoordinatDizi, HssKoordinat
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from typing import List, Dict, Optional, Tuple
import random

class Node3D:
    def __init__(self, x: float, y: float, z: float, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent
        self.cost = 0.0

class RRTPathPlanner(Node):
    def __init__(self):
        super().__init__('rrt_path_planner')
        
        # RRT Parameters
        self.step_size = 10.0  # metre
        self.max_iterations = 1000
        self.goal_threshold = 5.0  # metre
        
        # State variables
        self.start_point = None
        self.goal_point = None
        self.restricted_areas: List[Dict] = []
        self.nodes: List[Node3D] = []
        self.path_found = False
        self.current_path: List[Tuple[float, float, float]] = []
        
        # Publishers - Uçaktan yayınlanacak bilgiler
        self.path_pub = self.create_publisher(
            Path, 
            '/rrt/path', 
            10
        )
        
        self.tree_pub = self.create_publisher(
            MarkerArray, 
            '/rrt/tree', 
            10
        )
        
        self.obstacles_pub = self.create_publisher(
            MarkerArray, 
            '/rrt/obstacles', 
            10
        )
        
        # Subscribers
        # Tek topic dinleme - hss_koordinat (HssKoordinatDizi)
        self.hss_coordinate_sub = self.create_subscription(
            HssKoordinatDizi,
            '/hss_koordinat',
            self.hss_coordinate_callback,
            10
        )
        
        # Services (will be added later)
        
        # Timers
        self.visualization_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info('RRT Yol Planlayıcısı başlatıldı - hss_koordinat topic\'ini dinliyor (HssKoordinatDizi)')
        
    def hss_coordinate_callback(self, msg):
        """HSS koordinat topic'inden gelen verileri işler (HssKoordinatDizi)"""
        self.get_logger().info(f'HSS koordinat mesajı alındı: {len(msg.hss_koordinat_bilgileri)} adet koordinat')
        
        # Dizideki her koordinatı işle
        for hss_koordinat in msg.hss_koordinat_bilgileri:
            if hss_koordinat.id > 0:  # Yasaklı alan bilgilerini işle
                self.process_restricted_area(hss_koordinat)
            elif hss_koordinat.id == -1:  # Hedef nokta bilgisi
                self.process_target_point(hss_koordinat)
            
    def process_restricted_area(self, hss_koordinat):
        """Yasaklı alan bilgilerini işler (sadece daire şeklinde)"""
        area = {
            'id': hss_koordinat.id,
            'x': hss_koordinat.enlem,  # GPS koordinatlarını local koordinatlara çevir
            'y': hss_koordinat.boylam,
            'z': 0,  # Yer seviyesi (sadece daire şeklinde)
            'radius': hss_koordinat.yaricap,
            'is_active': hss_koordinat.is_active
        }
        
        if hss_koordinat.is_active:
            # Yeni alan ekle veya mevcut alanı güncelle
            existing_area = next((a for a in self.restricted_areas if a['id'] == hss_koordinat.id), None)
            if existing_area:
                existing_area.update(area)
                self.get_logger().info(f'Yasaklı alan güncellendi: ID={hss_koordinat.id}')
            else:
                self.restricted_areas.append(area)
                self.get_logger().info(f'Yeni yasaklı alan eklendi: ID={hss_koordinat.id}')
        else:
            # Alanı kaldır
            self.restricted_areas = [a for a in self.restricted_areas if a['id'] != hss_koordinat.id]
            self.get_logger().info(f'Yasaklı alan kaldırıldı: ID={hss_koordinat.id}')
            
    def process_target_point(self, hss_koordinat):
        """YKI'dan gelen hedef nokta bilgilerini işler"""
        # Mevcut konumu başlangıç noktası olarak kullan (gerçek uçak konumu)
        # Bu kısım uçak kontrolcüsünden alınacak
        current_pos = (0, 0, 50)  # Varsayılan değer
        
        # Hedef noktayı ayarla
        target_alt = hss_koordinat.yaricap if hss_koordinat.yaricap > 0 else 100
        goal_pos = (hss_koordinat.enlem, hss_koordinat.boylam, target_alt)
        
        self.set_start_goal(current_pos, goal_pos)
        
        # Yol planla
        path = self.plan_path()
        if path:
            self.get_logger().info(f'Hedef nokta için yol planlandı: {len(path)} nokta')
            self.publish_path(path)
        else:
            self.get_logger().warn('Hedef nokta için yol bulunamadı!')
            
    def set_start_goal(self, start: Tuple[float, float, float], goal: Tuple[float, float, float]):
        """Başlangıç ve hedef noktalarını ayarlar"""
        self.start_point = start
        self.goal_point = goal
        self.get_logger().info(f'Başlangıç: {start}, Hedef: {goal}')
        
    def plan_path(self) -> Optional[List[Tuple[float, float, float]]]:
        """RRT ile yol planlar"""
        if self.start_point is None or self.goal_point is None:
            self.get_logger().warn('Başlangıç veya hedef nokta ayarlanmamış!')
            return None
            
        # RRT ağacını sıfırla
        self.nodes = [Node3D(self.start_point[0], self.start_point[1], self.start_point[2])]
        self.path_found = False
        
        for i in range(self.max_iterations):
            # Rastgele nokta üret
            random_point = self.generate_random_point()
            
            # En yakın düğümü bul
            nearest_node = self.find_nearest_node(random_point)
            
            # Yeni düğüm oluştur
            new_point = self.steer(nearest_node, random_point)
            
            # Çarpışma kontrolü
            if not self.is_collision_free(nearest_node, new_point):
                continue
                
            # Yeni düğümü ekle
            new_node = Node3D(new_point[0], new_point[1], new_point[2], nearest_node)
            new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)
            self.nodes.append(new_node)
            
            # Hedefe ulaşıldı mı?
            if self.distance_to_goal(new_node) < self.goal_threshold:
                self.path_found = True
                self.get_logger().info(f'Yol bulundu! İterasyon: {i}')
                return self.extract_path(new_node)
                
        self.get_logger().warn('Maksimum iterasyon sayısına ulaşıldı, yol bulunamadı!')
        return None
        
    def generate_random_point(self) -> Tuple[float, float, float]:
        """Rastgele nokta üretir (hedef yönelimli)"""
        if random.random() < 0.1:  # %10 ihtimalle hedef noktayı seç
            return self.goal_point
            
        # Çalışma alanı sınırları (gerçek uygulamada dinamik olarak hesaplanmalı)
        x_min, x_max = -1000, 1000
        y_min, y_max = -1000, 1000
        z_min, z_max = 0, 200
        
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = random.uniform(z_min, z_max)
        
        return (x, y, z)
        
    def find_nearest_node(self, point: Tuple[float, float, float]) -> Node3D:
        """En yakın düğümü bulur"""
        min_distance = float('inf')
        nearest_node = None
        
        for node in self.nodes:
            dist = self.distance_point_to_node(point, node)
            if dist < min_distance:
                min_distance = dist
                nearest_node = node
                
        return nearest_node
        
    def steer(self, from_node: Node3D, to_point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Bir düğümden bir noktaya doğru adım atar"""
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dz = to_point[2] - from_node.z
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance <= self.step_size:
            return to_point
        else:
            # Normalize et ve step_size ile çarp
            scale = self.step_size / distance
            new_x = from_node.x + dx * scale
            new_y = from_node.y + dy * scale
            new_z = from_node.z + dz * scale
            return (new_x, new_y, new_z)
            
    def is_collision_free(self, from_node: Node3D, to_point: Tuple[float, float, float]) -> bool:
        """İki nokta arasındaki yolun çarpışma içerip içermediğini kontrol eder"""
        # Yol boyunca noktaları kontrol et
        n_checks = 10
        for i in range(n_checks + 1):
            t = i / n_checks
            x = from_node.x + t * (to_point[0] - from_node.x)
            y = from_node.y + t * (to_point[1] - from_node.y)
            z = from_node.z + t * (to_point[2] - from_node.z)
            
            if self.is_point_in_obstacle((x, y, z)):
                return False
                
        return True
        
    def is_point_in_obstacle(self, point: Tuple[float, float, float]) -> bool:
        """Bir noktanın engel içinde olup olmadığını kontrol eder (sadece daire)"""
        for area in self.restricted_areas:
            if not area['is_active']:
                continue
                
            # Dairesel engel kontrolü (sadece x,y koordinatları)
            distance = math.sqrt(
                (point[0] - area['x'])**2 + 
                (point[1] - area['y'])**2
            )
            
            if distance <= area['radius']:
                return True
                
        return False
        
    def distance(self, node1: Node3D, node2: Node3D) -> float:
        """İki düğüm arasındaki mesafeyi hesaplar"""
        return math.sqrt(
            (node1.x - node2.x)**2 + 
            (node1.y - node2.y)**2 + 
            (node1.z - node2.z)**2
        )
        
    def distance_point_to_node(self, point: Tuple[float, float, float], node: Node3D) -> float:
        """Bir nokta ile düğüm arasındaki mesafeyi hesaplar"""
        return math.sqrt(
            (point[0] - node.x)**2 + 
            (point[1] - node.y)**2 + 
            (point[2] - node.z)**2
        )
        
    def distance_to_goal(self, node: Node3D) -> float:
        """Düğümden hedefe olan mesafeyi hesaplar"""
        return self.distance_point_to_node(self.goal_point, node)
        
    def extract_path(self, goal_node: Node3D) -> List[Tuple[float, float, float]]:
        """Hedef düğümden başlayarak yolu çıkarır"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append((current.x, current.y, current.z))
            current = current.parent
            
        path.reverse()
        self.current_path = path
        return path
        
    def publish_path(self, path: List[Tuple[float, float, float]]):
        """Planlanan yolu yayınlar"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in path:
            pose = PoseStamped()
            # Float tipine dönüştür
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
    def publish_visualization(self):
        """Görselleştirme verilerini yayınlar"""
        # RRT ağacını görselleştir
        tree_markers = MarkerArray()
        
        for i, node in enumerate(self.nodes):
            if node.parent is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "rrt_tree"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # Çizgi rengi
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                
                # Çizgi kalınlığı
                marker.scale.x = 0.1
                
                # Çizgi noktaları
                start_point = Point()
                start_point.x = node.parent.x
                start_point.y = node.parent.y
                start_point.z = node.parent.z
                
                end_point = Point()
                end_point.x = node.x
                end_point.y = node.y
                end_point.z = node.z
                
                marker.points.append(start_point)
                marker.points.append(end_point)
                
                tree_markers.markers.append(marker)
                
        self.tree_pub.publish(tree_markers)
        
        # Engelleri görselleştir (sadece daire)
        obstacle_markers = MarkerArray()
        
        for i, area in enumerate(self.restricted_areas):
            if not area['is_active']:
                continue
                
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER  # Daire için silindir kullan
            marker.action = Marker.ADD
            
            # Engel pozisyonu
            marker.pose.position.x = area['x']
            marker.pose.position.y = area['y']
            marker.pose.position.z = area['z']
            
            # Engel boyutu (sadece daire)
            marker.scale.x = area['radius'] * 2
            marker.scale.y = area['radius'] * 2
            marker.scale.z = 10.0  # Sabit yükseklik
            
            # Engel rengi
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            
            obstacle_markers.markers.append(marker)
            
        self.obstacles_pub.publish(obstacle_markers)
        
        # Planlanan yolu görselleştir
        if self.current_path:
            self.publish_path(self.current_path)

def main(args=None):
    rclpy.init(args=args)
    
    planner = RRTPathPlanner()
    
    try:
        # Gerçek uçak modu - YKI'dan gelen verilere göre yol planla
        planner.get_logger().info('RRT yol planlayıcısı gerçek uçak modunda başlatıldı')
        
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info('RRT yol planlayıcısı kapatılıyor...')
        
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
