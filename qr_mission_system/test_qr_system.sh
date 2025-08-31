#!/bin/bash

# 🛩️ QR Kod Görevi Sistemi - Test Scripti
# Teknofest Projesi

echo "🛩️ QR Kod Görevi Sistemi Test Ediliyor..."
echo ""

# ROS2 environment'ını yükle
source /opt/ros/humble/setup.bash

# Workspace'i source et
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "📋 Test 1: QR sistemi node'ları çalışıyor mu?"
echo "-------------------------------------------"
ros2 node list | grep -E "(qr_dive_controller|qr_position_publisher)" || echo "❌ QR sistemi node'ları bulunamadı"

echo ""
echo "📋 Test 2: QR görev mesajı gönder"
echo "--------------------------------"
echo "QR kod dalış görevi başlatılıyor..."

ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
enlem: 41.0082
boylam: 28.9784
irtifa: 30.0
min_yukseklik: 30
baslangic_yuksekligi: 100
dalis_acisi: 15.0
is_active: true
emergency_stop: false
" --once

echo "✅ QR görev mesajı gönderildi"

echo ""
echo "📋 Test 3: QR pozisyon bilgilerini kontrol et"
echo "--------------------------------------------"
echo "5 saniye bekleniyor..."
sleep 5

echo "QR pozisyon bilgileri:"
ros2 topic echo /qr_mission/position --once 2>/dev/null || echo "❌ /qr_mission/position topic'i bulunamadı"

echo "QR koordinat bilgileri:"
ros2 topic echo /qr_mission/coordinates --once 2>/dev/null || echo "❌ /qr_mission/coordinates topic'i bulunamadı"

echo "QR dalış durumu:"
ros2 topic echo /qr_mission/dive_status --once 2>/dev/null || echo "❌ /qr_mission/dive_status topic'i bulunamadı"

echo ""
echo "📋 Test 4: Acil durdurma testi"
echo "-----------------------------"
echo "Acil durdurma komutu gönderiliyor..."

ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
enlem: 0.0
boylam: 0.0
irtifa: 0.0
min_yukseklik: 30
baslangic_yuksekligi: 100
dalis_acisi: 15.0
is_active: false
emergency_stop: true
" --once

echo "✅ Acil durdurma komutu gönderildi"

echo ""
echo "🎯 QR görev sistemi testi tamamlandı!"
echo "📊 Detaylı izleme için:"
echo "   ros2 topic echo /qr_mission/command"
echo "   ros2 topic echo /qr_mission/position"
echo "   ros2 topic echo /qr_mission/coordinates"
echo "   ros2 topic echo /qr_mission/dive_status"
echo "   ros2 topic echo /mavros/setpoint_velocity/cmd_vel"
