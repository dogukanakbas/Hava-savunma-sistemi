#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from air_defense_system.msg import HssKoordinatDizi, HssKoordinat
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np
import math
from typing import List, Dict, Optional
import time

class AircraftController(Node):
    def __init__(self):
        super().__init__('aircraft_controller')
        
        # State variables
        self.air_defense_active = False
        self.restricted_areas: List[Dict] = []
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.target_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.emergency_mode = False
        self.original_mission_active = True  # Normal görev durumu
        
        # Publishers - Uçaktan yayınlanacak bilgiler
        self.status_pub = self.create_publisher(
            String, 
            '/aircraft/status', 
            10
        )
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )
        
        self.target_velocity_pub = self.create_publisher(
            TwistStamped, 
            '/mavros/setpoint_velocity/cmd_vel', 
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
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.odom_callback,
            10
        )
        
        self.mavros_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            10
        )
        
        # Services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # RRT Path Planner
        self.path_planner = None  # Will be initialized when needed
        
        self.get_logger().info('Uçak kontrolcüsü başlatıldı - hss_koordinat topic\'ini dinliyor (HssKoordinatDizi)')
        
    def hss_coordinate_callback(self, msg):
        """HSS koordinat topic'inden gelen verileri işler (HssKoordinatDizi)"""
        self.get_logger().info(f'HSS koordinat mesajı alındı: {len(msg.hss_koordinat_bilgileri)} adet koordinat')
        
        # Dizideki her koordinatı işle
        for hss_koordinat in msg.hss_koordinat_bilgileri:
            if hss_koordinat.id > 0:
                # Yasaklı alan bilgisi
                self.process_restricted_area(hss_koordinat)
            else:
                # Komut bilgisi (aktif/pasif)
                self.process_command(hss_koordinat)
            
    def process_restricted_area(self, hss_koordinat):
        """Yasaklı alan bilgilerini işler"""
        area = {
            'id': hss_koordinat.id,
            'lat': hss_koordinat.enlem,
            'lon': hss_koordinat.boylam,
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
            
    def process_command(self, hss_koordinat):
        """Komut bilgilerini işler"""
        # is_active alanına göre HSS aktif/pasif durumunu belirle
        if hss_koordinat.is_active:
            if not self.air_defense_active:
                self.air_defense_active = True
                self.original_mission_active = False  # Normal görevi duraklat
                self.get_logger().info('Hava savunma sistemi aktifleştirildi - Normal görev duraklatıldı')
        else:
            if self.air_defense_active:
                self.air_defense_active = False
                self.original_mission_active = True  # Normal görevi devam ettir
                self.get_logger().info('Hava savunma sistemi deaktifleştirildi - Normal göreve dönülüyor')
                # Tüm yasaklı alanları temizle
                self.restricted_areas.clear()
                self.get_logger().info('Tüm yasaklı alanlar temizlendi')
            
    def gps_callback(self, msg):
        """GPS verilerini alır"""
        self.current_position['lat'] = msg.latitude
        self.current_position['lon'] = msg.longitude
        self.current_position['alt'] = msg.altitude
        
    def odom_callback(self, msg):
        """Odometri verilerini alır"""
        # Local position bilgilerini güncelle
        pass
        
    def mavros_state_callback(self, msg):
        """MAVROS durum bilgilerini alır"""
        # Uçak durumunu takip et
        pass
        
    def is_in_restricted_area(self, lat: float, lon: float) -> bool:
        """Verilen konumun yasaklı alanda olup olmadığını kontrol eder (sadece daire)"""
        for area in self.restricted_areas:
            if not area['is_active']:
                continue
                
            # Mesafe hesapla (Haversine formülü)
            distance = self.calculate_distance(
                lat, lon, area['lat'], area['lon']
            )
            
            # Yasaklı alan içinde mi? (sadece yatay mesafe kontrolü)
            if distance <= area['radius']:
                return True
                
        return False
        
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """İki GPS koordinatı arasındaki mesafeyi hesaplar (metre)"""
        R = 6371000  # Dünya yarıçapı (metre)
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) * math.sin(delta_lat / 2) +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) * math.sin(delta_lon / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
        
    def find_safe_path(self, target_lat: float, target_lon: float) -> Optional[Dict]:
        """Güvenli yol bulur (sadece daire şeklinde engeller için)"""
        if not self.air_defense_active:
            return {'lat': target_lat, 'lon': target_lon, 'alt': self.current_position['alt']}
            
        # Hedef noktaya doğrudan gitmeyi dene
        if not self.is_in_restricted_area(target_lat, target_lon):
            return {'lat': target_lat, 'lon': target_lon, 'alt': self.current_position['alt']}
            
        # Yasaklı alan varsa, etrafından dolan
        for area in self.restricted_areas:
            if not area['is_active']:
                continue
                
            # Yasaklı alanın etrafından geç
            safe_distance = area['radius'] + 50  # 50m güvenlik mesafesi
            
            # En yakın güvenli noktayı bul
            angle = math.atan2(target_lat - area['lat'], target_lon - area['lon'])
            safe_lat = area['lat'] + safe_distance * math.cos(angle) / 111000  # Yaklaşık dönüşüm
            safe_lon = area['lon'] + safe_distance * math.sin(angle) / (111000 * math.cos(math.radians(area['lat'])))
            
            if not self.is_in_restricted_area(safe_lat, safe_lon):
                return {'lat': safe_lat, 'lon': safe_lon, 'alt': self.current_position['alt']}
                
        return None
        
    def control_loop(self):
        """Ana kontrol döngüsü"""
        if self.emergency_mode:
            self.execute_emergency_landing()
            return
            
        if not self.air_defense_active:
            # Normal görev modunda
            if self.original_mission_active:
                self.execute_normal_mission()
            return
            
        # Hava savunma modunda
        # Mevcut konumun yasaklı alanda olup olmadığını kontrol et
        if self.is_in_restricted_area(
            self.current_position['lat'],
            self.current_position['lon']
        ):
            self.get_logger().warn('Yasaklı alanda bulunuyor! Acil kaçınma manevrası başlatılıyor...')
            self.execute_emergency_avoidance()
            
    def execute_normal_mission(self):
        """Normal görev prosedürü"""
        # Burada normal görev kodları olacak
        # Örneğin: waypoint takibi, görev yürütme vb.
        pass
        
    def execute_emergency_avoidance(self):
        """Acil kaçınma manevrası"""
        # En yakın güvenli noktaya git
        safe_point = self.find_nearest_safe_point()
        if safe_point:
            self.send_position_command(safe_point['lat'], safe_point['lon'], safe_point['alt'])
            
    def find_nearest_safe_point(self) -> Optional[Dict]:
        """En yakın güvenli noktayı bulur"""
        # Mevcut konumdan başlayarak dışarı doğru spiral arama
        current_lat = self.current_position['lat']
        current_lon = self.current_position['lon']
        current_alt = self.current_position['alt']
        
        for radius in range(100, 1000, 50):  # 100m'den 1000m'ye kadar
            for angle in range(0, 360, 30):  # 30 derece adımlarla
                test_lat = current_lat + radius * math.cos(math.radians(angle)) / 111000
                test_lon = current_lon + radius * math.sin(math.radians(angle)) / (111000 * math.cos(math.radians(current_lat)))
                
                if not self.is_in_restricted_area(test_lat, test_lon):
                    return {'lat': test_lat, 'lon': test_lon, 'alt': current_alt}
                    
        return None
        
    def execute_emergency_landing(self):
        """Acil iniş prosedürü"""
        # Güvenli iniş noktasına git
        landing_lat = self.current_position['lat']
        landing_lon = self.current_position['lon']
        landing_alt = 0  # Yere iniş
        
        self.send_position_command(landing_lat, landing_lon, landing_alt)
        
    def send_position_command(self, lat: float, lon: float, alt: float):
        """Pozisyon komutu gönderir"""
        # GPS koordinatlarını local koordinatlara çevir
        # Bu kısım MAVROS ile entegre edilecek
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        
        # Basit dönüşüm (gerçek uygulamada daha hassas olmalı)
        pose.pose.position.x = (lon - self.current_position['lon']) * 111000
        pose.pose.position.y = (lat - self.current_position['lat']) * 111000
        pose.pose.position.z = alt - self.current_position['alt']
        
        self.target_pose_pub.publish(pose)
        
    def publish_status(self):
        """Durum bilgisini yayınlar"""
        status = "NORMAL_MISSION"
        if self.emergency_mode:
            status = "EMERGENCY"
        elif self.air_defense_active:
            status = "AIR_DEFENSE_ACTIVE"
        elif not self.original_mission_active:
            status = "MISSION_PAUSED"
            
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = AircraftController()
    
    try:
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Uçak kontrolcüsü kapatılıyor...')
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
