# 🎯 Otonom Hedef Takip Sistemi

## 📋 Genel Bakış

Otonom Hedef Takip Sistemi, sabit kanat İHA'lar için hedef uçak takip görevini gerçekleştiren ROS2 paketidir. Bu sistem 10m yakın mesafede hedefin arkasında takip yapar.

## 🎯 Özellikler

- **10m Yakın Takip**: Çok yakın mesafede takip
- **Hedefin Arkasında**: Güvenli pozisyon
- **Aynı Yükseklik**: Hedefle aynı irtifa
- **Güvenlik Kontrolleri**: Minimum/maksimum mesafe
- **Hedef Kaybolma**: Otomatik arama
- **Acil Durdurma**: Güvenlik için

## 📁 Dosya Yapısı

```
target_tracking_system/
├── package.xml
├── CMakeLists.txt
├── README.md
├── msg/
│   └── TargetInfo.msg
├── scripts/
│   └── target_tracking_controller.py
├── launch/
│   └── target_tracking_system.launch.py
└── test_tracking_system.sh
```

## 🚀 Kurulum ve Çalıştırma

### 1. Kurulum
```bash
# Workspace'e kopyala
cp -r target_tracking_system ~/ros2_ws/src/

# Derle
cd ~/ros2_ws
colcon build --packages-select target_tracking_system
source install/setup.bash
```

### 2. Sistemi Başlatma
```bash
# Takip sistemi başlat
ros2 launch target_tracking_system target_tracking_system.launch.py
```

### 3. Test Etme
```bash
# Otomatik test
./target_tracking_system/test_tracking_system.sh

# Manuel test
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

## 📊 Topic Yapısı

### Giriş Topic'leri
- `/target_tracking/target_info` → Hedef bilgileri (TargetInfo)

### Çıkış Topic'leri
- `/target_tracking/status` → Takip durumu (String)
- `/mavros/setpoint_position/local` → Pozisyon komutları (PoseStamped)

## 📝 Mesaj Tipleri

### TargetInfo.msg
```msg
Header header
bool kitlenme
int32 takim_id
float64 enlem
float64 boylam
float64 irtifa
float32 takip_mesafesi
bool takip_aktif
bool acil_durdurma
```

## 🎯 Takip Parametreleri

### Mesafe Ayarları
- **Takip Mesafesi**: 10m (varsayılan)
- **Minimum Güvenlik**: 5m
- **Maksimum Takip**: 100m
- **Yaklaşma Mesafesi**: 50m

### Yükseklik Ayarları
- **Takip Yüksekliği**: Hedefle aynı
- **Güvenlik Yüksekliği**: Hedefin 20m üstü

## 🛩️ Takip Akışı

### 1. Arama Modu (SEARCHING)
- Hedef tespit edilir
- 50m yaklaşma mesafesine gelir
- Hedef bulunursa yaklaşma moduna geçer

### 2. Yaklaşma Modu (APPROACHING)
- Hedefi yakalamaya çalışır
- 10m takip mesafesine ulaşır
- Hedef yakalanırsa takip moduna geçer

### 3. Takip Modu (TRACKING)
- Hedefin arkasında 10m mesafede takip eder
- Hedefle aynı yükseklikte uçar
- Sürekli mesafe kontrolü yapar

### 4. Güvenlik Kontrolleri
- 5m'den yakın → Uzaklaş
- 100m'den uzak → Yaklaş
- Hedef kaybolursa → Arama modu

## 🧪 Test Senaryoları

### Senaryo 1: Normal Takip
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

### Senaryo 2: Acil Durdurma
```bash
# Acil durdurma komutu
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

### Senaryo 3: Hedef Kaybolma
```bash
# Hedef bilgilerini durdur
ros2 topic pub /target_tracking/target_info target_tracking_system/msg/TargetInfo "
kitlenme: false
takim_id: 0
enlem: 0.0
boylam: 0.0
irtifa: 0.0
takip_mesafesi: 10.0
takip_aktif: true
acil_durdurma: false
"
```

## 📈 İzleme Komutları

### Topic'leri Dinle
```bash
# Hedef bilgileri
ros2 topic echo /target_tracking/target_info

# Takip durumu
ros2 topic echo /target_tracking/status

# Pozisyon komutları
ros2 topic echo /mavros/setpoint_position/local

# Hız komutları
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

### Sistem Durumu
```bash
# Node'ları listele
ros2 node list

# Topic'leri listele
ros2 topic list

# Node bilgilerini gör
ros2 node info /target_tracking_controller
```

## 🔧 Konfigürasyon

### Takip Parametreleri (Kodda Değiştirilebilir)
```python
self.tracking_distance = 10.0      # Takip mesafesi (metre)
self.approach_distance = 50.0      # Yaklaşma mesafesi (metre)
self.min_safe_distance = 5.0       # Minimum güvenlik (metre)
self.max_tracking_distance = 100.0 # Maksimum takip (metre)
self.target_lost_timeout = 5.0     # Hedef kaybolma süresi (saniye)
```

### Hız Parametreleri
```python
# Yaklaşma hızı
velocity.twist.linear.x = 15.0  # m/s

# Takip hızı
velocity.twist.linear.x = 20.0  # m/s

# Güvenlik hızı
velocity.twist.linear.x = 10.0  # m/s
```

## ⚠️ Güvenlik Özellikleri

### Mesafe Kontrolleri
- **Minimum Mesafe**: 5m (çarpışma önleme)
- **Maksimum Mesafe**: 100m (kaybetme önleme)
- **Yaklaşma Mesafesi**: 50m (güvenli yaklaşma)

### Zaman Kontrolleri
- **Hedef Kaybolma**: 5 saniye
- **Kontrol Frekansı**: 10Hz
- **Durum Yayını**: 1Hz

### Acil Durumlar
- **Acil Durdurma**: `acil_durdurma = true`
- **Hedef Kaybolma**: Otomatik arama modu
- **Güvenlik İhlali**: Otomatik uzaklaşma

## 🆘 Sorun Giderme

### Yaygın Hatalar
- **"Package not found"**: `source install/setup.bash` çalıştırın
- **"Topic not found"**: Node'ların çalıştığından emin olun
- **"MAVROS connection failed"**: Pixhawk bağlantısını kontrol edin

### Debug Modu
```bash
# Debug log'ları ile çalıştır
ros2 run target_tracking_system target_tracking_controller --ros-args --log-level debug
```

### Log Seviyeleri
```bash
# Info seviyesi
ros2 run target_tracking_system target_tracking_controller --ros-args --log-level info

# Warning seviyesi
ros2 run target_tracking_system target_tracking_controller --ros-args --log-level warn

# Error seviyesi
ros2 run target_tracking_system target_tracking_controller --ros-args --log-level error
```

## 📊 Performans

### Kontrol Frekansları
- **Ana Kontrol Döngüsü**: 10Hz
- **Durum Yayını**: 1Hz
- **Pozisyon Komutları**: 10Hz

### Mesafe Hesaplama
- **GPS Koordinatları**: Haversine formülü
- **Hassasiyet**: ±1m
- **Güncelleme**: Her kontrol döngüsünde

### Hız Kontrolü
- **Maksimum Hız**: 25 m/s
- **Minimum Hız**: 10 m/s
- **Takip Hızı**: Hedefle uyumlu

## 🔄 Entegrasyon

### Diğer Sistemlerle
- **Hava Savunma Sistemi**: Bağımsız çalışır
- **QR Görev Sistemi**: Bağımsız çalışır
- **MAVROS**: Pixhawk ile entegre

### Topic Çakışması
- **Pozisyon Komutları**: `/mavros/setpoint_position/local`
- **Hız Komutları**: `/mavros/setpoint_velocity/cmd_vel`
- **Durum Bilgileri**: `/target_tracking/status`

## 📞 Destek

### Sorun Yaşarsanız
1. Log'ları kontrol edin
2. Topic'lerin çalıştığını doğrulayın
3. MAVROS bağlantısını test edin
4. Test scriptini çalıştırın

### İletişim
- **E-posta**: teknofest@example.com
- **Dokümantasyon**: Bu README dosyası
- **Test Scripti**: `test_tracking_system.sh`

## 📝 Notlar

### Önemli Hatırlatmalar
- Sistem 10m yakın takip yapar
- Hedefin arkasında pozisyon alır
- Hedefle aynı yükseklikte uçar
- Güvenlik kontrolleri aktif
- Acil durdurma özelliği mevcut

### Geliştirme Önerileri
- Hedef hareket yönü tahmini
- Daha gelişmiş takip algoritması
- Çoklu hedef takibi
- Görsel geri bildirim
