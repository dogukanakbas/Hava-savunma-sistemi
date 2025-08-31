#!/bin/bash

# Hava Savunma Sistemi - Kurulum ve Çalıştırma Komutları
# Teknofest Projesi

echo "🚁 Hava Savunma Sistemi - Kurulum Başlıyor..."

# 1. ROS2 Workspace'e git
cd ~/ros2_ws

# 2. Paketi derle
echo "📦 Paket derleniyor..."
colcon build --packages-select air_defense_system

# 3. Source et
echo "🔧 Environment yükleniyor..."
source install/setup.bash

echo "✅ Kurulum tamamlandı!"
echo ""
echo "🎯 Çalıştırma komutları:"
echo "1. Sistemi başlat: ros2 launch air_defense_system air_defense_system.launch.py"
echo "2. Tek tek başlat:"
echo "   - Uçak kontrolcüsü: ros2 run air_defense_system aircraft_controller"
echo "   - RRT planlayıcı: ros2 run air_defense_system rrt_path_planner"
echo ""
echo "📊 Topic'leri izle:"
echo "   - ros2 topic list"
echo "   - ros2 topic echo /hss_koordinat"
echo "   - ros2 topic echo /aircraft/status"
echo "   - ros2 topic echo /rrt/path"
