#!/bin/bash

# 🎯 Otonom Hedef Takip Sistemi - Test Scripti
# Teknofest Projesi

echo "🎯 Otonom Hedef Takip Sistemi Test Ediliyor..."
echo ""

# ROS2 environment'ını yükle
source /opt/ros/humble/setup.bash

# Workspace'i source et
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "📋 Test 1: Takip sistemi node'ları çalışıyor mu?"
echo "---------------------------------------------"
ros2 node list | grep target_tracking_controller || echo "❌ Takip sistemi node'ları bulunamadı"

echo ""
echo "📋 Test 2: Hedef takip mesajı gönder"
echo "-----------------------------------"
echo "Hedef takip görevi başlatılıyor..."

ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
kitlenme: true
takim_id: 123
enlem: 41.0082
boylam: 28.9784
irtifa: 100.0
takip_mesafesi: 10.0
takip_aktif: true
acil_durdurma: false
" --once

echo "✅ Hedef takip mesajı gönderildi"

echo ""
echo "📋 Test 3: Takip durumunu kontrol et"
echo "-----------------------------------"
echo "5 saniye bekleniyor..."
sleep 5

echo "Takip durumu:"
ros2 topic echo /target_tracking/status --once 2>/dev/null || echo "❌ /target_tracking/status topic'i bulunamadı"

echo ""
echo "📋 Test 4: Acil durdurma testi"
echo "-----------------------------"
echo "Acil durdurma komutu gönderiliyor..."

ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
kitlenme: false
takim_id: 0
enlem: 0.0
boylam: 0.0
irtifa: 0.0
takip_mesafesi: 10.0
takip_aktif: false
acil_durdurma: true
" --once

echo "✅ Acil durdurma komutu gönderildi"

echo ""
echo "🎯 Hedef takip sistemi testi tamamlandı!"
echo "📊 Detaylı izleme için:"
echo "   ros2 topic echo /target_tracking/target_info"
echo "   ros2 topic echo /target_tracking/status"
echo "   ros2 topic echo /mavros/setpoint_position/local"
