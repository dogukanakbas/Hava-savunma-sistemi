#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from qr_mission_system.msg import QrMission
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np
import math
from typing import Dict, Optional
import time
from enum import Enum

class QrMissionState(Enum):
    IDLE = "IDLE"
    APPROACHING = "APPROACHING"
    DIVING = "DIVING"
    READING = "READING"
    ASCENDING = "ASCENDING"
    COMPLETED = "COMPLETED"
    EMERGENCY = "EMERGENCY"

class QrDiveController(Node):
    def __init__(self):
        super().__init__('qr_dive_controller')
        
        # State variables
        self.mission_state = QrMissionState.IDLE
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.qr_target = None
        self.dive_params = {
            'min_altitude': 30,
            'start_altitude': 100,
            'dive_angle': 15.0
        }
        self.mission_active = False
        self.emergency_stop = False
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 
            '/qr_mission/dive_status', 
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
        self.qr_mission_sub = self.create_subscription(
            QrMission,
            '/qr_mission/command',
            self.qr_mission_callback,
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
        
        # Mission variables
        self.approach_distance = 200  # metre - yaklaşma mesafesi
        self.dive_start_time = None
        self.dive_duration = 15  # saniye - maksimum dalış süresi
        self.reading_duration = 5  # saniye - okuma süresi
        
        self.get_logger().info('QR Dalış Kontrolcüsü başlatıldı')
        
    def qr_mission_callback(self, msg):
        """QR görev komutlarını işler"""
        if msg.emergency_stop:
            self.emergency_stop_mission()
            return
            
        if msg.is_active and not self.mission_active:
            # Yeni görev başlat
            self.start_qr_mission(msg)
        elif not msg.is_active and self.mission_active:
            # Görevi durdur
            self.stop_qr_mission()
            
    def start_qr_mission(self, msg):
        """QR görevini başlatır"""
        self.qr_target = {
            'lat': msg.enlem,
            'lon': msg.boylam,
            'alt': msg.irtifa
        }
        
        self.dive_params = {
            'min_altitude': msg.min_yukseklik,
            'start_altitude': msg.baslangic_yuksekligi,
            'dive_angle': msg.dalis_acisi
        }
        
        self.mission_active = True
        self.emergency_stop = False
        self.mission_state = QrMissionState.APPROACHING
        
        self.get_logger().info(f'QR görevi başlatıldı: {self.qr_target}')
        self.get_logger().info(f'Dalış parametreleri: {self.dive_params}')
        
    def stop_qr_mission(self):
        """QR görevini durdurur"""
        self.mission_active = False
        self.mission_state = QrMissionState.IDLE
        self.qr_target = None
        self.dive_start_time = None
        
        self.get_logger().info('QR görevi durduruldu')
        
    def emergency_stop_mission(self):
        """Acil durdurma"""
        self.emergency_stop = True
        self.mission_state = QrMissionState.EMERGENCY
        
        # Hemen yükselme komutu gönder
        self.send_emergency_ascend()
        self.get_logger().error('QR görevi acil durduruldu!')
        
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
        if not self.mission_active or self.emergency_stop:
            return
            
        if self.mission_state == QrMissionState.APPROACHING:
            self.execute_approach()
        elif self.mission_state == QrMissionState.DIVING:
            self.execute_dive()
        elif self.mission_state == QrMissionState.READING:
            self.execute_reading()
        elif self.mission_state == QrMissionState.ASCENDING:
            self.execute_ascend()
            
    def execute_approach(self):
        """QR kod konumuna yaklaşma"""
        if not self.qr_target:
            return
            
        # QR kod konumuna olan mesafeyi hesapla
        distance = self.calculate_distance(
            self.current_position['lat'],
            self.current_position['lon'],
            self.qr_target['lat'],
            self.qr_target['lon']
        )
        
        if distance <= self.approach_distance:
            # Yaklaşma tamamlandı, dalışa geç
            self.mission_state = QrMissionState.DIVING
            self.dive_start_time = time.time()
            self.get_logger().info('Yaklaşma tamamlandı, dalış başlatılıyor...')
        else:
            # QR kod konumuna git
            self.send_position_command(
                self.qr_target['lat'],
                self.qr_target['lon'],
                self.dive_params['start_altitude']
            )
            
    def execute_dive(self):
        """Kontrollü dalış manevrası"""
        if not self.dive_start_time:
            return
            
        elapsed_time = time.time() - self.dive_start_time
        
        # Dalış süresi kontrolü
        if elapsed_time > self.dive_duration:
            self.mission_state = QrMissionState.READING
            self.get_logger().info('Dalış tamamlandı, okuma moduna geçiliyor...')
            return
            
        # Minimum yükseklik kontrolü
        if self.current_position['alt'] <= self.dive_params['min_altitude']:
            self.mission_state = QrMissionState.READING
            self.get_logger().info('Minimum yüksekliğe ulaşıldı, okuma moduna geçiliyor...')
            return
            
        # Dalış manevrası - hız komutu gönder
        self.send_dive_velocity_command()
        
    def execute_reading(self):
        """QR kod okuma modu (sabit kanat için kısa süre)"""
        if not self.dive_start_time:
            return
            
        elapsed_time = time.time() - self.dive_start_time
        reading_start_time = self.dive_start_time + self.dive_duration
        
        if time.time() - reading_start_time >= self.reading_duration:
            # Okuma tamamlandı, yükselmeye geç
            self.mission_state = QrMissionState.ASCENDING
            self.get_logger().info('Okuma tamamlandı, yükselme başlatılıyor...')
        else:
            # Okuma sırasında pozisyonu koru
            self.send_position_command(
                self.qr_target['lat'],
                self.qr_target['lon'],
                self.current_position['alt']
            )
            
    def execute_ascend(self):
        """Güvenli yükselme manevrası"""
        # Güvenli yüksekliğe yüksel
        target_alt = self.dive_params['start_altitude']
        
        if self.current_position['alt'] >= target_alt:
            # Yükselme tamamlandı
            self.mission_state = QrMissionState.COMPLETED
            self.get_logger().info('QR görevi başarıyla tamamlandı!')
            self.stop_qr_mission()
        else:
            # Yükselme komutu gönder
            self.send_ascend_velocity_command()
            
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
        
    def send_dive_velocity_command(self):
        """Dalış hız komutu gönderir"""
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = "map"
        
        # Dalış açısına göre hız ayarla
        dive_angle_rad = math.radians(self.dive_params['dive_angle'])
        
        # İleri hız (sabit kanat için yüksek)
        velocity.twist.linear.x = 20.0  # m/s
        
        # Dikey hız (dalış)
        velocity.twist.linear.z = -20.0 * math.sin(dive_angle_rad)
        
        self.target_velocity_pub.publish(velocity)
        
    def send_ascend_velocity_command(self):
        """Yükselme hız komutu gönderir"""
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = "map"
        
        # Yükselme açısına göre hız ayarla
        ascend_angle_rad = math.radians(20.0)  # 20 derece yükselme
        
        # İleri hız
        velocity.twist.linear.x = 15.0  # m/s
        
        # Dikey hız (yükselme)
        velocity.twist.linear.z = 15.0 * math.sin(ascend_angle_rad)
        
        self.target_velocity_pub.publish(velocity)
        
    def send_emergency_ascend(self):
        """Acil yükselme komutu"""
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = "map"
        
        # Maksimum yükselme hızı
        velocity.twist.linear.x = 10.0  # m/s
        velocity.twist.linear.z = 10.0  # m/s (dik yükselme)
        
        self.target_velocity_pub.publish(velocity)
        
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
        status_msg.data = self.mission_state.value
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = QrDiveController()
    
    try:
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('QR Dalış Kontrolcüsü kapatılıyor...')
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
