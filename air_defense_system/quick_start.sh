#!/bin/bash

# 🚁 Hava Savunma Sistemi - Hızlı Başlangıç
# Teknofest Projesi

echo "🚁 Hava Savunma Sistemi Başlatılıyor..."
echo ""

# ROS2 environment'ını yükle
source /opt/ros/humble/setup.bash

# Workspace'i source et (eğer varsa)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace yüklendi"
fi

# MAVROS'u başlat (eğer Pixhawk bağlıysa)
echo "🔌 MAVROS bağlantısı kontrol ediliyor..."
if ros2 topic list | grep -q "mavros"; then
    echo "✅ MAVROS topic'leri bulundu"
else
    echo "⚠️  MAVROS topic'leri bulunamadı - Pixhawk bağlantısını kontrol edin"
fi

echo ""
echo "🎯 Sistem başlatılıyor..."
echo "📊 Topic'leri izlemek için:"
echo "   ros2 topic echo /hss_koordinat"
echo "   ros2 topic echo /aircraft/status"
echo "   ros2 topic echo /rrt/path"
echo ""

# Sistemi başlat
ros2 launch air_defense_system air_defense_system.launch.py
