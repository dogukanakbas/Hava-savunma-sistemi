# 🚁 Hava Savunma Sistemi - Çalıştırma Kılavuzu

## 📋 Ön Gereksinimler

### 1. ROS2 Kurulumu
```bash
# Ubuntu 22.04 için ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# ROS2 environment'ını yükle
source /opt/ros/humble/setup.bash
```

### 2. Gerekli Paketler
```bash
# MAVROS ve MAVLink
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Python paketleri
pip3 install numpy matplotlib pymavlink
```

### 3. Workspace Kurulumu
```bash
# ROS2 workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Paketi kopyala
cp -r /path/to/air_defense_system ./

# Workspace'e dön ve derle
cd ~/ros2_ws
colcon build --packages-select air_defense_system
source install/setup.bash
```

## 🚀 Sistem Çalıştırma

### Yöntem 1: Launch File ile (Önerilen)
```bash
# Tüm sistemi tek seferde başlat
ros2 launch air_defense_system air_defense_system.launch.py
```

### Yöntem 2: Manuel Başlatma
```bash
# Terminal 1: Uçak Kontrolcüsü
ros2 run air_defense_system aircraft_controller

# Terminal 2: RRT Yol Planlayıcısı  
ros2 run air_defense_system rrt_path_planner

# Terminal 3: RViz2 (Görselleştirme için)
rviz2
```

## 📊 Sistem İzleme

### Topic'leri Listele
```bash
# Mevcut topic'leri gör
ros2 topic list

# Topic bilgilerini gör
ros2 topic info /hss_koordinat
ros2 topic info /aircraft/status
ros2 topic info /rrt/path
```

### Topic'leri Dinle
```bash
# HSS koordinat mesajlarını dinle
ros2 topic echo /hss_koordinat

# Uçak durumunu dinle
ros2 topic echo /aircraft/status

# Planlanan yolu dinle
ros2 topic echo /rrt/path

# MAVROS durumunu dinle
ros2 topic echo /mavros/state
```

### Node'ları İzle
```bash
# Çalışan node'ları listele
ros2 node list

# Node bilgilerini gör
ros2 node info /aircraft_controller
ros2 node info /rrt_path_planner
```

## 🧪 Test Senaryoları

### 1. Manuel Test Mesajı Gönderme
```bash
# Test için HssKoordinatDizi mesajı gönder
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
- id: 0
  enlem: 0.0
  boylam: 0.0
  yaricap: 0
  is_active: true
"
```

### 2. YKI Entegrasyonu Testi
```bash
# YKI'nızın gönderdiği mesajları dinle
ros2 topic echo /hss_koordinat --once

# Sistem yanıtını kontrol et
ros2 topic echo /aircraft/status --once
```

## 🔧 Hata Ayıklama

### Log'ları İzle
```bash
# Tüm log'ları gör
ros2 run air_defense_system aircraft_controller --ros-args --log-level debug

# Belirli node'un log'larını gör
ros2 run air_defense_system rrt_path_planner --ros-args --log-level info
```

### Sistem Durumu Kontrolü
```bash
# MAVROS bağlantısını kontrol et
ros2 topic echo /mavros/state --once

# GPS verilerini kontrol et
ros2 topic echo /mavros/global_position/global --once
```

## 📈 Performans İzleme

### CPU ve Bellek Kullanımı
```bash
# Sistem kaynaklarını izle
htop

# ROS2 node'larının kaynak kullanımını gör
ros2 run system_monitor system_monitor
```

### Network Trafiği
```bash
# Topic trafiğini izle
ros2 topic hz /hss_koordinat
ros2 topic bw /hss_koordinat
```

## 🛑 Sistemi Durdurma

### Güvenli Durdurma
```bash
# Ctrl+C ile node'ları durdur
# Veya
ros2 node kill /aircraft_controller
ros2 node kill /rrt_path_planner
```

### Acil Durdurma
```bash
# Tüm ROS2 süreçlerini durdur
pkill -f ros2
```

## 🔄 Otomatik Başlatma

### Systemd Service (Ubuntu)
```bash
# Service dosyası oluştur
sudo nano /etc/systemd/system/air-defense-system.service

# Service'i etkinleştir
sudo systemctl enable air-defense-system.service
sudo systemctl start air-defense-system.service
```

### Cron Job
```bash
# Sistem başlangıcında otomatik başlat
crontab -e
# @reboot cd ~/ros2_ws && source install/setup.bash && ros2 launch air_defense_system air_defense_system.launch.py
```

## 📝 Önemli Notlar

1. **MAVROS Bağlantısı**: Pixhawk bağlantısı için MAVROS'un çalıştığından emin olun
2. **GPS Sinyali**: Uçak GPS sinyali almadan sistem çalışmaz
3. **YKI Bağlantısı**: `/hss_koordinat` topic'ine YKI'nızdan veri gelmeli
4. **Güvenlik**: Test ortamında uçak motorlarını devre dışı bırakın
5. **Log'lar**: Hata durumunda log'ları kontrol edin

## 🆘 Sorun Giderme

### Yaygın Hatalar
- **"Package not found"**: `source install/setup.bash` çalıştırın
- **"Topic not found"**: Node'ların çalıştığından emin olun
- **"MAVROS connection failed"**: Pixhawk bağlantısını kontrol edin
- **"Permission denied"**: USB port izinlerini kontrol edin

### Destek
Sorun yaşarsanız:
1. Log'ları kontrol edin
2. Topic'lerin çalıştığını doğrulayın
3. MAVROS bağlantısını test edin
4. YKI mesaj formatını kontrol edin
