#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from qr_mission_system.msg import QrMission
import math
from typing import Dict, Optional
import time

class QrPositionPublisher(Node):
    def __init__(self):
        super().__init__('qr_position_publisher')
        
        # State variables
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.qr_target = None
        self.mission_active = False
        
        # Publishers - QR görevinden yayınlanacak bilgiler
        self.qr_position_pub = self.create_publisher(
            NavSatFix, 
            '/qr_mission/position', 
            10
        )
        
        self.qr_status_pub = self.create_publisher(
            String, 
            '/qr_mission/status', 
            10
        )
        
        self.qr_coordinates_pub = self.create_publisher(
            String, 
            '/qr_mission/coordinates', 
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
        
        # Timers
        self.position_timer = self.create_timer(1.0, self.publish_position)
        self.coordinates_timer = self.create_timer(0.5, self.publish_coordinates)
        
        self.get_logger().info('QR Pozisyon Yayınlayıcısı başlatıldı')
        
    def qr_mission_callback(self, msg):
        """QR görev komutlarını işler"""
        if msg.is_active and not self.mission_active:
            # Yeni görev başlat
            self.qr_target = {
                'lat': msg.enlem,
                'lon': msg.boylam,
                'alt': msg.irtifa
            }
            self.mission_active = True
            self.get_logger().info(f'QR görevi başlatıldı: {self.qr_target}')
            
        elif not msg.is_active and self.mission_active:
            # Görevi durdur
            self.mission_active = False
            self.qr_target = None
            self.get_logger().info('QR görevi durduruldu')
            
    def gps_callback(self, msg):
        """GPS verilerini alır"""
        self.current_position['lat'] = msg.latitude
        self.current_position['lon'] = msg.longitude
        self.current_position['alt'] = msg.altitude
        
    def publish_position(self):
        """QR görev pozisyon bilgilerini yayınlar"""
        if not self.mission_active or not self.qr_target:
            return
            
        # QR hedef pozisyonunu yayınla
        position_msg = NavSatFix()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = "map"
        
        position_msg.latitude = self.qr_target['lat']
        position_msg.longitude = self.qr_target['lon']
        position_msg.altitude = self.qr_target['alt']
        
        self.qr_position_pub.publish(position_msg)
        
    def publish_coordinates(self):
        """Enlem, boylam, irtifa bilgilerini string olarak yayınlar"""
        if not self.mission_active or not self.qr_target:
            return
            
        # Koordinat bilgilerini string formatında yayınla
        coordinates_msg = String()
        coordinates_msg.data = f"Enlem: {self.qr_target['lat']:.6f}, Boylam: {self.qr_target['lon']:.6f}, İrtifa: {self.qr_target['alt']:.2f}m"
        
        self.qr_coordinates_pub.publish(coordinates_msg)
        
        # Durum bilgisini de yayınla
        status_msg = String()
        status_msg.data = "QR_MISSION_ACTIVE"
        self.qr_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = QrPositionPublisher()
    
    try:
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        publisher.get_logger().info('QR Pozisyon Yayınlayıcısı kapatılıyor...')
        
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
