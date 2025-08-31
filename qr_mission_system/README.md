# 🛩️ QR Kod Dalış Görevi Sistemi

## 📋 Genel Bakış

QR Kod Dalış Görevi Sistemi, sabit kanat İHA'lar için QR kod okuma görevini gerçekleştiren ROS2 paketidir. Bu sistem 30m'ye kadar dalış yaparak QR kodu okumaya çalışır.

## 🎯 Özellikler

- **Sabit Kanat Optimizasyonu**: Durma olmadan süzülme dalışı
- **30m Dalış**: Minimum yükseklik kontrolü
- **Hızlı Okuma**: 5-10 saniye okuma süresi
- **Güvenli Yükselme**: Otomatik yükselme manevrası
- **Acil Durdurma**: Güvenlik için
- **Pozisyon Yayını**: Enlem, boylam, irtifa bilgileri

## 📁 Dosya Yapısı

```
qr_mission_system/
├── package.xml
├── CMakeLists.txt
├── README.md
├── QUICK_START.md
├── msg/
│   └── QrMission.msg
├── scripts/
│   ├── qr_dive_controller.py
│   └── qr_position_publisher.py
├── launch/
│   └── qr_mission_system.launch.py
└── test_qr_system.sh
```

## 🚀 Kurulum ve Çalıştırma

### 1. Kurulum
```bash
# Workspace'e kopyala
cp -r qr_mission_system ~/ros2_ws/src/

# Derle
cd ~/ros2_ws
colcon build --packages-select qr_mission_system
source install/setup.bash
```

### 2. Sistemi Başlatma
```bash
# QR görev sistemi başlat
ros2 launch qr_mission_system qr_mission_system.launch.py
```

### 3. Test Etme
```bash
# Otomatik test
./qr_mission_system/test_qr_system.sh

# Manuel test
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

## 📊 Topic Yapısı

### Giriş Topic'leri
- `/qr_mission/command` → QR görev komutları (QrMission)

### Çıkış Topic'leri
- `/qr_mission/position` → QR hedef pozisyonu (NavSatFix)
- `/qr_mission/coordinates` → Enlem, boylam, irtifa (String)
- `/qr_mission/dive_status` → Dalış durumu (String)
- `/mavros/setpoint_position/local` → Pozisyon komutları (PoseStamped)
- `/mavros/setpoint_velocity/cmd_vel` → Hız komutları (TwistStamped)

## 📝 Mesaj Tipleri

### QrMission.msg
```msg
Header header
float64 enlem
float64 boylam
float64 irtifa
int32 min_yukseklik
int32 baslangic_yuksekligi
float32 dalis_acisi
bool is_active
bool emergency_stop
```

## 🛩️ Dalış Parametreleri

### Yükseklik Ayarları
- **Başlangıç Yüksekliği**: 100m (varsayılan)
- **Minimum Dalış**: 30m
- **Güvenlik Yüksekliği**: 25m (acil yükselme)

### Dalış Parametreleri
- **Dalış Açısı**: 15° (varsayılan)
- **Yükselme Açısı**: 20°
- **Yaklaşma Mesafesi**: 200m

### Zaman Parametreleri
- **Maksimum Dalış Süresi**: 15 saniye
- **Okuma Süresi**: 5 saniye
- **Kontrol Frekansı**: 10Hz

## 🎯 Dalış Akışı

### 1. Yaklaşma (APPROACHING)
- QR kod konumuna 100m yükseklikte yaklaşır
- 200m mesafeye geldiğinde dalışa geçer
- Hedef konuma doğru yönelir

### 2. Dalış (DIVING)
- 15° açıyla kontrollü dalış
- 30m minimum yüksekliğe kadar
- Maksimum 15 saniye dalış süresi
- Hız kontrolü ile güvenli dalış

### 3. Okuma (READING)
- 30m'de 5 saniye QR kodu okumaya çalışır
- Sabit kanat için kısa süre
- Pozisyonu korur
- QR tespiti ayrı sistemde yapılır

### 4. Yükselme (ASCENDING)
- 20° açıyla güvenli yükselme
- 100m'ye geri döner
- Güvenli yüksekliğe ulaşır

## 🧪 Test Senaryoları

### Senaryo 1: Normal QR Görevi
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

### Senaryo 2: Acil Durdurma
```bash
# Acil durdurma komutu
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
# Farklı dalış parametreleri
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

## 📈 İzleme Komutları

### Topic'leri Dinle
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

### Sistem Durumu
```bash
# Node'ları listele
ros2 node list

# Topic'leri listele
ros2 topic list

# Node bilgilerini gör
ros2 node info /qr_dive_controller
ros2 node info /qr_position_publisher
```

## 🔧 Konfigürasyon

### Dalış Parametreleri (Kodda Değiştirilebilir)
```python
self.approach_distance = 200      # Yaklaşma mesafesi (metre)
self.dive_duration = 15           # Maksimum dalış süresi (saniye)
self.reading_duration = 5         # Okuma süresi (saniye)
self.dive_angle = 15.0            # Dalış açısı (derece)
self.ascend_angle = 20.0          # Yükselme açısı (derece)
```

### Hız Parametreleri
```python
# Dalış hızı
velocity.twist.linear.x = 20.0  # m/s (ileri)
velocity.twist.linear.z = -20.0 * math.sin(dive_angle_rad)  # m/s (dalış)

# Yükselme hızı
velocity.twist.linear.x = 15.0  # m/s (ileri)
velocity.twist.linear.z = 15.0 * math.sin(ascend_angle_rad)  # m/s (yükselme)

# Acil yükselme hızı
velocity.twist.linear.x = 10.0  # m/s
velocity.twist.linear.z = 10.0  # m/s (dik yükselme)
```

## ⚠️ Güvenlik Özellikleri

### Yükseklik Kontrolleri
- **Minimum Yükseklik**: 30m (dalış sınırı)
- **Güvenlik Yüksekliği**: 25m (acil yükselme)
- **Başlangıç Yüksekliği**: 100m (güvenli)

### Zaman Kontrolleri
- **Maksimum Dalış**: 15 saniye
- **Okuma Süresi**: 5 saniye
- **Kontrol Frekansı**: 10Hz

### Acil Durumlar
- **Acil Durdurma**: `emergency_stop = true`
- **Minimum Yükseklik**: Otomatik yükselme
- **Maksimum Süre**: Otomatik yükselme

## 🆘 Sorun Giderme

### Yaygın Hatalar
- **"Package not found"**: `source install/setup.bash` çalıştırın
- **"Topic not found"**: Node'ların çalıştığından emin olun
- **"MAVROS connection failed"**: Pixhawk bağlantısını kontrol edin

### Debug Modu
```bash
# Debug log'ları ile çalıştır
ros2 run qr_mission_system qr_dive_controller --ros-args --log-level debug
```

### Log Seviyeleri
```bash
# Info seviyesi
ros2 run qr_mission_system qr_dive_controller --ros-args --log-level info

# Warning seviyesi
ros2 run qr_mission_system qr_dive_controller --ros-args --log-level warn

# Error seviyesi
ros2 run qr_mission_system qr_dive_controller --ros-args --log-level error
```

## 📊 Performans

### Kontrol Frekansları
- **Ana Kontrol Döngüsü**: 10Hz
- **Pozisyon Yayını**: 1Hz
- **Koordinat Yayını**: 2Hz
- **Durum Yayını**: 1Hz

### Dalış Performansı
- **Dalış Süresi**: 10-15 saniye
- **Okuma Süresi**: 5 saniye
- **Yükselme Süresi**: 10-15 saniye
- **Toplam Süre**: 25-35 saniye

### Hız Kontrolü
- **Dalış Hızı**: 20 m/s
- **Yükselme Hızı**: 15 m/s
- **Acil Yükselme**: 10 m/s

## 🔄 Entegrasyon

### Diğer Sistemlerle
- **Hava Savunma Sistemi**: Bağımsız çalışır
- **Hedef Takip Sistemi**: Bağımsız çalışır
- **MAVROS**: Pixhawk ile entegre

### QR Tespit Sistemi
- **QR Okuma**: Ayrı sistem tarafından yapılır
- **Bu Sistem**: Sadece dalış ve yönelme
- **Entegrasyon**: Topic üzerinden veri alışverişi

### Topic Çakışması
- **Pozisyon Komutları**: `/mavros/setpoint_position/local`
- **Hız Komutları**: `/mavros/setpoint_velocity/cmd_vel`
- **Durum Bilgileri**: `/qr_mission/dive_status`

## 📞 Destek

### Sorun Yaşarsanız
1. Log'ları kontrol edin
2. Topic'lerin çalıştığını doğrulayın
3. MAVROS bağlantısını test edin
4. Test scriptini çalıştırın

### İletişim
- **E-posta**: teknofest@example.com
- **Dokümantasyon**: Bu README dosyası
- **Test Scripti**: `test_qr_system.sh`

## 📝 Notlar

### Önemli Hatırlatmalar
- Sistem sabit kanat için optimize edilmiştir
- QR tespiti ayrı sistemde yapılır
- 30m minimum yükseklik kontrolü aktif
- Acil durdurma özelliği mevcut
- Güvenli yükselme manevrası

### Geliştirme Önerileri
- QR tespit sistemi entegrasyonu
- Daha gelişmiş dalış algoritması
- Çoklu QR kod desteği
- Görsel geri bildirim
- Dalış performans optimizasyonu
