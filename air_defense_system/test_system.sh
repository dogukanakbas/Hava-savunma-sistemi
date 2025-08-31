#!/bin/bash

# 🧪 Hava Savunma Sistemi - Test Scripti
# Teknofest Projesi

echo "🧪 Hava Savunma Sistemi Test Ediliyor..."
echo ""

# ROS2 environment'ını yükle
source /opt/ros/humble/setup.bash

# Workspace'i source et
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "📋 Test 1: Topic'lerin varlığını kontrol et"
echo "----------------------------------------"
ros2 topic list | grep -E "(hss_koordinat|aircraft|rrt|mavros)" || echo "❌ Topic'ler bulunamadı"

echo ""
echo "📋 Test 2: Node'ların çalışmasını kontrol et"
echo "-------------------------------------------"
ros2 node list | grep -E "(aircraft_controller|rrt_path_planner)" || echo "❌ Node'lar bulunamadı"

echo ""
echo "📋 Test 3: Test mesajı gönder"
echo "----------------------------"
echo "Test HssKoordinatDizi mesajı gönderiliyor..."

ros2 topic pub /hss_koordinat air_defense_system/msg/HssKoordinatDizi "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
hss_koordinat_bilgileri:
- id: 1
  enlem: 41.0082
  boylam: 28.9784
  yaricap: 100
  is_active: true
- id: 2
  enlem: 41.0182
  boylam: 28.9884
  yaricap: 150
  is_active: true
- id: 0
  enlem: 0.0
  boylam: 0.0
  yaricap: 0
  is_active: true
" --once

echo "✅ Test mesajı gönderildi"

echo ""
echo "📋 Test 4: Sistem yanıtını kontrol et"
echo "------------------------------------"
echo "5 saniye bekleniyor..."
sleep 5

echo "Uçak durumu:"
ros2 topic echo /aircraft/status --once 2>/dev/null || echo "❌ /aircraft/status topic'i bulunamadı"

echo "RRT yolu:"
ros2 topic echo /rrt/path --once 2>/dev/null || echo "❌ /rrt/path topic'i bulunamadı"

echo ""
echo "📋 Test 5: MAVROS bağlantısı"
echo "---------------------------"
if ros2 topic list | grep -q "mavros"; then
    echo "✅ MAVROS topic'leri mevcut"
    ros2 topic echo /mavros/state --once 2>/dev/null || echo "⚠️  MAVROS state bilgisi alınamadı"
else
    echo "❌ MAVROS topic'leri bulunamadı - Pixhawk bağlantısını kontrol edin"
fi

echo ""
echo "🎯 Test tamamlandı!"
echo "📊 Detaylı izleme için:"
echo "   ros2 topic echo /hss_koordinat"
echo "   ros2 topic echo /aircraft/status"
echo "   ros2 topic echo /rrt/path"
