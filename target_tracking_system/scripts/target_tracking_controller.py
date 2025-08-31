#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from target_tracking_system.msg import TargetInfo
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np
import math
from typing import Dict, Optional
import time
from enum import Enum

class TrackingState(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    TRACKING = "TRACKING"
    LOST_TARGET = "LOST_TARGET"
    EMERGENCY = "EMERGENCY"

class TargetTrackingController(Node):
    def __init__(self):
        super().__init__('target_tracking_controller')
        
        # State variables
        self.tracking_state = TrackingState.IDLE
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.target_info = None
        self.tracking_active = False
        self.emergency_stop = False
        
        # Takip parametreleri - ÇOK YAKIN TAKİP
        self.tracking_distance = 10.0  # metre - ÇOK YAKIN
        self.approach_distance = 50.0  # metre - yaklaşma mesafesi
        self.min_safe_distance = 5.0   # metre - minimum güvenli mesafe
        self.max_tracking_distance = 100.0  # metre - maksimum takip mesafesi
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 
            '/target_tracking/status', 
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
        self.target_info_sub = self.create_subscription(
            TargetInfo,
            '/target_tracking/target_info',
            self.target_info_callback,
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
        
        # Tracking variables
        self.last_target_time = None
        self.target_lost_timeout = 5.0  # saniye - hedef kaybolma süresi
        
        self.get_logger().info('Hedef Takip Kontrolcüsü başlatıldı - 10m yakın takip')
        
    def target_info_callback(self, msg):
        """Hedef bilgilerini işler"""
        if msg.acil_durdurma:
            self.emergency_stop_tracking()
            return
            
        if msg.takip_aktif and not self.tracking_active:
            # Takip başlat
            self.start_tracking(msg)
        elif not msg.takip_aktif and self.tracking_active:
            # Takibi durdur
            self.stop_tracking()
            
        # Hedef bilgilerini güncelle
        if msg.kitlenme:
            self.target_info = {
                'takim_id': msg.takim_id,
                'lat': msg.enlem,
                'lon': msg.boylam,
                'alt': msg.irtifa,
                'tracking_distance': msg.takip_mesafesi if msg.takip_mesafesi > 0 else self.tracking_distance
            }
            self.last_target_time = time.time()
            
    def start_tracking(self, msg):
        """Hedef takibini başlatır"""
        self.tracking_active = True
        self.emergency_stop = False
        self.tracking_state = TrackingState.SEARCHING
        
        self.get_logger().info(f'Hedef takibi başlatıldı - Takım ID: {msg.takim_id}')
        
    def stop_tracking(self):
        """Hedef takibini durdurur"""
        self.tracking_active = False
        self.tracking_state = TrackingState.IDLE
        self.target_info = None
        self.last_target_time = None
        
        self.get_logger().info('Hedef takibi durduruldu')
        
    def emergency_stop_tracking(self):
        """Acil durdurma"""
        self.emergency_stop = True
        self.tracking_state = TrackingState.EMERGENCY
        
        # Güvenli bölgeye git
        self.send_safe_position_command()
        self.get_logger().error('Hedef takibi acil durduruldu!')
        
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
        
    def control_loop(self):
        """Ana kontrol döngüsü"""
        if not self.tracking_active or self.emergency_stop:
            return
            
        # Hedef kaybolma kontrolü
        if self.last_target_time and (time.time() - self.last_target_time) > self.target_lost_timeout:
            self.tracking_state = TrackingState.LOST_TARGET
            self.get_logger().warn('Hedef kayboldu! Arama moduna geçiliyor...')
            return
            
        if self.tracking_state == TrackingState.SEARCHING:
            self.execute_searching()
        elif self.tracking_state == TrackingState.APPROACHING:
            self.execute_approaching()
        elif self.tracking_state == TrackingState.TRACKING:
            self.execute_tracking()
        elif self.tracking_state == TrackingState.LOST_TARGET:
            self.execute_lost_target()
            
    def execute_searching(self):
        """Hedef arama modu"""
        if self.target_info:
            # Hedef bulundu, yaklaşma moduna geç
            distance = self.calculate_distance_to_target()
            if distance <= self.approach_distance:
                self.tracking_state = TrackingState.APPROACHING
                self.get_logger().info('Hedef yaklaşma mesafesinde, takip moduna geçiliyor...')
            else:
                # Hedefi yakalamak için git
                self.send_approach_command()
                
    def execute_approaching(self):
        """Hedef yakalama modu"""
        if not self.target_info:
            return
            
        distance = self.calculate_distance_to_target()
        
        if distance <= self.tracking_distance:
            # Takip mesafesine ulaştı, takip moduna geç
            self.tracking_state = TrackingState.TRACKING
            self.get_logger().info(f'Hedef yakalandı! Takip mesafesi: {distance:.1f}m')
        else:
            # Hedefi yakalamaya devam et
            self.send_approach_command()
            
    def execute_tracking(self):
        """Yakın takip modu - 10m mesafede"""
        if not self.target_info:
            return
            
        distance = self.calculate_distance_to_target()
        
        # Güvenlik kontrolü
        if distance < self.min_safe_distance:
            # Çok yakın, uzaklaş
            self.get_logger().warn(f'Çok yakın! Mesafe: {distance:.1f}m, uzaklaşılıyor...')
            self.send_backoff_command()
        elif distance > self.max_tracking_distance:
            # Çok uzak, yaklaş
            self.get_logger().warn(f'Çok uzak! Mesafe: {distance:.1f}m, yaklaşılıyor...')
            self.send_approach_command()
        else:
            # İdeal takip mesafesinde, hedefin arkasında takip et
            self.send_tracking_command()
            
    def execute_lost_target(self):
        """Hedef kaybolma modu"""
        # Hedefi aramak için son bilinen konuma git
        if self.target_info:
            self.send_search_command()
        else:
            # Hedef bilgisi yok, güvenli bölgeye git
            self.send_safe_position_command()
            
    def calculate_distance_to_target(self) -> float:
        """Hedef uçağa olan mesafeyi hesaplar"""
        if not self.target_info:
            return float('inf')
            
        return self.calculate_distance(
            self.current_position['lat'],
            self.current_position['lon'],
            self.target_info['lat'],
            self.target_info['lon']
        )
        
    def send_approach_command(self):
        """Hedefi yakalamak için komut gönderir"""
        if not self.target_info:
            return
            
        # Hedefin arkasında, aynı yükseklikte pozisyon hesapla
        target_pos = self.calculate_tracking_position()
        
        self.send_position_command(
            target_pos['lat'],
            target_pos['lon'],
            self.target_info['alt']  # Hedefle aynı yükseklik
        )
        
    def send_tracking_command(self):
        """Yakın takip komutu gönderir - 10m mesafede"""
        if not self.target_info:
            return
            
        # Hedefin arkasında 10m mesafede pozisyon hesapla
        target_pos = self.calculate_tracking_position()
        
        self.send_position_command(
            target_pos['lat'],
            target_pos['lon'],
            self.target_info['alt']  # Hedefle aynı yükseklik
        )
        
    def send_backoff_command(self):
        """Uzaklaşma komutu gönderir"""
        if not self.target_info:
            return
            
        # Hedeften uzaklaş
        target_pos = self.calculate_backoff_position()
        
        self.send_position_command(
            target_pos['lat'],
            target_pos['lon'],
            self.target_info['alt']
        )
        
    def send_search_command(self):
        """Arama komutu gönderir"""
        if not self.target_info:
            return
            
        # Son bilinen hedef konumuna git
        self.send_position_command(
            self.target_info['lat'],
            self.target_info['lon'],
            self.target_info['alt']
        )
        
    def send_safe_position_command(self):
        """Güvenli pozisyon komutu gönderir"""
        # Mevcut konumda kal veya güvenli bölgeye git
        self.send_position_command(
            self.current_position['lat'],
            self.current_position['lon'],
            self.current_position['alt']
        )
        
    def calculate_tracking_position(self) -> Dict:
        """Hedefin arkasında takip pozisyonu hesaplar"""
        if not self.target_info:
            return self.current_position
            
        # Hedefin arkasında 10m mesafede pozisyon hesapla
        # Basit hesaplama: Hedefin 10m gerisinde
        tracking_distance = self.target_info.get('tracking_distance', self.tracking_distance)
        
        # Hedefin arkasında pozisyon (basit yaklaşım)
        # Gerçek uygulamada hedefin hareket yönü de hesaba katılmalı
        lat_offset = tracking_distance / 111000  # Yaklaşık dönüşüm
        
        tracking_lat = self.target_info['lat'] - lat_offset
        tracking_lon = self.target_info['lon']
        
        return {
            'lat': tracking_lat,
            'lon': tracking_lon
        }
        
    def calculate_backoff_position(self) -> Dict:
        """Uzaklaşma pozisyonu hesaplar"""
        if not self.target_info:
            return self.current_position
            
        # Hedeften 15m uzakta pozisyon
        backoff_distance = 15.0
        lat_offset = backoff_distance / 111000
        
        backoff_lat = self.target_info['lat'] - lat_offset
        backoff_lon = self.target_info['lon']
        
        return {
            'lat': backoff_lat,
            'lon': backoff_lon
        }
        
    def send_position_command(self, lat: float, lon: float, alt: float):
        """Pozisyon komutu gönderir"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        
        # GPS koordinatlarını local koordinatlara çevir
        pose.pose.position.x = float((lon - self.current_position['lon']) * 111000)
        pose.pose.position.y = float((lat - self.current_position['lat']) * 111000)
        pose.pose.position.z = float(alt - self.current_position['alt'])
        
        self.target_pose_pub.publish(pose)
        
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
        
    def publish_status(self):
        """Durum bilgisini yayınlar"""
        status_msg = String()
        status_msg.data = self.tracking_state.value
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = TargetTrackingController()
    
    try:
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Hedef Takip Kontrolcüsü kapatılıyor...')
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
