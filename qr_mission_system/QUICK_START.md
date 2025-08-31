# 🚀 QR Kod Dalış Görevi - Hızlı Başlangıç

## ⚡ 5 Dakikada Çalıştırma

### 1. Kurulum (1 dakika)
```bash
# Workspace'e kopyala
cp -r qr_mission_system ~/ros2_ws/src/

# Derle
cd ~/ros2_ws
colcon build --packages-select qr_mission_system
source install/setup.bash
```

### 2. Sistemi Başlat (30 saniye)
```bash
# QR görev sistemi başlat
ros2 launch qr_mission_system qr_mission_system.launch.py
```

### 3. Test Et (30 saniye)
```bash
# Yeni terminal aç ve test et
./qr_mission_system/test_qr_system.sh
```

### 4. Manuel Test (1 dakika)
```bash
# QR görev başlat
ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
enlem: 41.0082
boylam: 28.9784
irtifa: 30.0
min_yukseklik: 30
baslangic_yuksekligi: 100
dalis_acisi: 15.0
is_active: true
emergency_stop: false
"

# Durumu izle
ros2 topic echo /qr_mission/dive_status
ros2 topic echo /qr_mission/coordinates
```

## 🎯 Temel Komutlar

### Sistemi Başlatma
```bash
ros2 launch qr_mission_system qr_mission_system.launch.py
```

### Test Etme
```bash
./qr_mission_system/test_qr_system.sh
```

### Topic'leri İzleme
```bash
# QR görev komutları
ros2 topic echo /qr_mission/command

# QR pozisyon bilgileri
ros2 topic echo /qr_mission/position

# Koordinat bilgileri
ros2 topic echo /qr_mission/coordinates

# Dalış durumu
ros2 topic echo /qr_mission/dive_status

# Pozisyon komutları
ros2 topic echo /mavros/setpoint_position/local

# Hız komutları
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

### Sistemi Durdurma
```bash
# Ctrl+C ile durdur
# Veya
ros2 node kill /qr_dive_controller
ros2 node kill /qr_position_publisher
```

## 📊 Hızlı Test Senaryoları

### Senaryo 1: Normal QR Görevi
```bash
ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
enlem: 41.0082
boylam: 28.9784
irtifa: 30.0
min_yukseklik: 30
baslangic_yuksekligi: 100
dalis_acisi: 15.0
is_active: true
emergency_stop: false
"
```

### Senaryo 2: Acil Durdurma
```bash
ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
enlem: 0.0
boylam: 0.0
irtifa: 0.0
min_yukseklik: 30
baslangic_yuksekligi: 100
dalis_acisi: 15.0
is_active: false
emergency_stop: true
"
```

### Senaryo 3: Farklı Parametreler
```bash
ros2 topic pub /qr_mission/command qr_mission_system/msg/QrMission "
enlem: 41.0182
boylam: 28.9884
irtifa: 25.0
min_yukseklik: 25
baslangic_yuksekligi: 80
dalis_acisi: 20.0
is_active: true
emergency_stop: false
"
```

## ⚠️ Hızlı Sorun Giderme

### Problem: "Package not found"
```bash
# Çözüm
source ~/ros2_ws/install/setup.bash
```

### Problem: "Topic not found"
```bash
# Çözüm
ros2 node list  # Node'ların çalıştığını kontrol et
ros2 topic list # Topic'lerin varlığını kontrol et
```

### Problem: "MAVROS connection failed"
```bash
# Çözüm
ros2 topic echo /mavros/state  # MAVROS durumunu kontrol et
```

## 🎯 Önemli Parametreler

### Dalış Parametreleri
- **Başlangıç Yüksekliği**: 100m
- **Minimum Dalış**: 30m
- **Dalış Açısı**: 15°
- **Yükselme Açısı**: 20°

### Zaman Parametreleri
- **Maksimum Dalış**: 15 saniye
- **Okuma Süresi**: 5 saniye
- **Toplam Süre**: 25-35 saniye

### Durum Mesajları
- `IDLE`: Bekleme
- `APPROACHING`: Yaklaşıyor
- `DIVING`: Dalış yapıyor
- `READING`: QR okuyor
- `ASCENDING`: Yükseliyor
- `COMPLETED`: Tamamlandı
- `EMERGENCY`: Acil durum

## 📈 Performans Bilgileri

### Hız Parametreleri
- **Dalış Hızı**: 20 m/s
- **Yükselme Hızı**: 15 m/s
- **Acil Yükselme**: 10 m/s

### Kontrol Frekansları
- **Ana Kontrol**: 10Hz
- **Pozisyon Yayını**: 1Hz
- **Koordinat Yayını**: 2Hz

## 🔧 Hızlı Konfigürasyon

### Dalış Parametrelerini Değiştirme
```python
# qr_dive_controller.py dosyasında
self.approach_distance = 200      # Yaklaşma mesafesi
self.dive_duration = 15           # Maksimum dalış süresi
self.reading_duration = 5         # Okuma süresi
self.dive_angle = 15.0            # Dalış açısı
```

### Hız Parametrelerini Değiştirme
```python
# Dalış hızı
velocity.twist.linear.x = 20.0  # m/s

# Yükselme hızı
velocity.twist.linear.x = 15.0  # m/s
```

## 📞 Hızlı Destek

### Sorun Yaşarsanız
1. `./qr_mission_system/test_qr_system.sh` çalıştırın
2. Log'ları kontrol edin
3. Topic'lerin çalıştığını doğrulayın

### Detaylı Bilgi
- **Tam Dokümantasyon**: `README.md`
- **Test Scripti**: `test_qr_system.sh`
- **Launch Dosyası**: `launch/qr_mission_system.launch.py`

## 🎯 Önemli Notlar

### Güvenlik
- 30m minimum yükseklik kontrolü aktif
- Acil durdurma özelliği mevcut
- Otomatik yükselme manevrası

### Sabit Kanat Optimizasyonu
- Durma olmadan süzülme dalışı
- Hızlı geçiş sırasında okuma
- Güvenli yükselme manevrası

### QR Tespit
- QR okuma ayrı sistemde yapılır
- Bu sistem sadece dalış ve yönelme
- Topic üzerinden entegrasyon
