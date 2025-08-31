# 🚀 Otonom Takip Sistemi - Hızlı Başlangıç

## ⚡ 5 Dakikada Çalıştırma

### 1. Kurulum (1 dakika)
```bash
# Workspace'e kopyala
cp -r target_tracking_system ~/ros2_ws/src/

# Derle
cd ~/ros2_ws
colcon build --packages-select target_tracking_system
source install/setup.bash
```

### 2. Sistemi Başlat (30 saniye)
```bash
# Takip sistemi başlat
ros2 launch target_tracking_system target_tracking_system.launch.py
```

### 3. Test Et (30 saniye)
```bash
# Yeni terminal aç ve test et
./target_tracking_system/test_tracking_system.sh
```

### 4. Manuel Test (1 dakika)
```bash
# Hedef bilgileri gönder
ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
kitlenme: true
takim_id: 123
enlem: 41.0082
boylam: 28.9784
irtifa: 100.0
takip_mesafesi: 10.0
takip_aktif: true
acil_durdurma: false
"

# Durumu izle
ros2 topic echo /target_tracking/status
```

## 🎯 Temel Komutlar

### Sistemi Başlatma
```bash
ros2 launch target_tracking_system target_tracking_system.launch.py
```

### Test Etme
```bash
./target_tracking_system/test_tracking_system.sh
```

### Topic'leri İzleme
```bash
# Hedef bilgileri
ros2 topic echo /target_tracking/target_info

# Takip durumu
ros2 topic echo /target_tracking/status

# Pozisyon komutları
ros2 topic echo /mavros/setpoint_position/local
```

### Sistemi Durdurma
```bash
# Ctrl+C ile durdur
# Veya
ros2 node kill /target_tracking_controller
```

## 📊 Hızlı Test Senaryoları

### Senaryo 1: Normal Takip
```bash
ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
kitlenme: true
takim_id: 123
enlem: 41.0082
boylam: 28.9784
irtifa: 100.0
takip_mesafesi: 10.0
takip_aktif: true
acil_durdurma: false
"
```

### Senaryo 2: Acil Durdurma
```bash
ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
kitlenme: false
takim_id: 0
enlem: 0.0
boylam: 0.0
irtifa: 0.0
takip_mesafesi: 10.0
takip_aktif: false
acil_durdurma: true
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

### Takip Mesafeleri
- **Takip Mesafesi**: 10m
- **Minimum Güvenlik**: 5m
- **Maksimum Takip**: 100m

### Durum Mesajları
- `IDLE`: Bekleme
- `SEARCHING`: Hedef arıyor
- `APPROACHING`: Yaklaşıyor
- `TRACKING`: Takip ediyor
- `LOST_TARGET`: Hedef kayboldu
- `EMERGENCY`: Acil durum

## 📞 Hızlı Destek

### Sorun Yaşarsanız
1. `./target_tracking_system/test_tracking_system.sh` çalıştırın
2. Log'ları kontrol edin
3. Topic'lerin çalıştığını doğrulayın

### Detaylı Bilgi
- **Tam Dokümantasyon**: `README.md`
- **Test Scripti**: `test_tracking_system.sh`
- **Launch Dosyası**: `launch/target_tracking_system.launch.py`
