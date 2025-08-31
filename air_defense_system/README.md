# Teknofest Hava Savunma Sistemi

Bu proje, Teknofest yarışması için geliştirilmiş ROS2 tabanlı hava savunma sistemidir. Sistem, yasaklı uçuş alanlarını tespit ederek UAV'lerin bu alanlardan kaçınmasını sağlar.

## Sistem Mimarisi

### Bileşenler

1. **Yer Kontrol İstasyonu (Ground Control Station)**
   - Sunucudan yasaklı alan bilgilerini alır
   - Hava savunma komutlarını yayınlar
   - Uçak durumunu takip eder

2. **Uçak Kontrolcüsü (Aircraft Controller)**
   - Pixhawk Orange Cube ile iletişim kurar
   - Yasaklı alanlardan kaçınma algoritması
   - Acil durum prosedürleri

3. **RRT Yol Planlayıcısı (RRT Path Planner)**
   - Rapidly-exploring Random Tree algoritması
   - 3D yol planlama
   - Engelden kaçınma

### Donanım Gereksinimleri

- **Yer Kontrol İstasyonu**: Linux bilgisayar
- **Uçak**: 
  - Pixhawk Orange Cube
  - NVIDIA Jetson Nano
  - GPS modülü
  - Telemetri sistemi

## Kurulum

### Gereksinimler

```bash
# ROS2 Humble kurulumu
sudo apt update
sudo apt install ros-humble-desktop

# Gerekli paketler
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo apt install python3-pip
pip3 install numpy matplotlib
```

### Proje Kurulumu

```bash
# Workspace oluştur
mkdir -p ~/air_defense_ws/src
cd ~/air_defense_ws/src

# Projeyi klonla
git clone <repository_url> air_defense_system

# Bağımlılıkları yükle
cd ~/air_defense_ws
rosdep install --from-paths src --ignore-src -r -y

# Projeyi derle
colcon build

# Environment'ı yükle
source install/setup.bash
```

## Kullanım

### Sistem Başlatma

```bash
# Tüm sistemi başlat
ros2 launch air_defense_system air_defense_system.launch.py

# Sadece yer kontrol istasyonu
ros2 run air_defense_system ground_control_station.py

# Sadece uçak kontrolcüsü
ros2 run air_defense_system aircraft_controller.py

# Sadece RRT yol planlayıcısı
ros2 run air_defense_system rrt_path_planner.py
```

### Topic'ler

#### Yayınlanan Topic'ler
- `/air_defense/command` - Hava savunma komutları
- `/air_defense/restricted_areas` - Yasaklı alan bilgileri
- `/aircraft/status` - Uçak durumu
- `/rrt/path` - Planlanan yol
- `/rrt/tree` - RRT ağacı görselleştirmesi
- `/rrt/obstacles` - Engel görselleştirmesi

#### Dinlenen Topic'ler
- `/mavros/global_position/global` - GPS verileri
- `/mavros/state` - MAVROS durumu
- `/mavros/setpoint_position/local` - Pozisyon komutları

### Mesaj Tipleri

#### RestrictedArea.msg
```msg
Header header
float64 latitude
float64 longitude
float64 altitude
float64 radius
float64 height
bool is_active
uint32 area_id
uint8 area_type
geometry_msgs/Point32[] polygon_points
```

#### AirDefenseCommand.msg
```msg
Header header
uint8 command_type
string[] parameters
uint8 priority
time timestamp
```

## Konfigürasyon

### MAVROS Konfigürasyonu

Pixhawk bağlantısı için MAVROS konfigürasyonu:

```bash
# MAVROS başlat
ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyACM0:115200

# Uçuş modunu ayarla
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "custom_mode: 'GUIDED'"

# Silahlandır
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "value: true"
```

### RRT Parametreleri

RRT algoritması parametreleri `rrt_path_planner.py` dosyasında ayarlanabilir:

```python
self.step_size = 10.0  # metre
self.max_iterations = 1000
self.goal_threshold = 5.0  # metre
```

## Test

### Simülasyon Testi

```bash
# Gazebo simülasyonu ile test
ros2 launch air_defense_system air_defense_system.launch.py use_sim_time:=true

# Test verileri gönder
ros2 topic pub /air_defense/restricted_areas air_defense_system/msg/RestrictedArea "{area_id: 1, latitude: 40.9867, longitude: 29.0304, altitude: 100, radius: 500, height: 200, is_active: true}"
```

### Gerçek Uçuş Testi

1. Pixhawk'ı bağla ve MAVROS'u başlat
2. GPS sinyalini kontrol et
3. Sistemi başlat
4. Test yasaklı alanları gönder
5. Uçağın kaçınma davranışını gözlemle

## Güvenlik

### Acil Durum Prosedürleri

- **Acil İniş**: Sistem otomatik olarak güvenli iniş noktasına yönlendirir
- **Yasaklı Alan Tespiti**: Uçak otomatik olarak güvenli bölgeye kaçar
- **İletişim Kesintisi**: Son bilinen güvenli noktaya döner

### Güvenlik Önlemleri

- Maksimum yükseklik sınırı
- Minimum güvenlik mesafesi
- Batarya seviyesi kontrolü
- İletişim durumu takibi

## Geliştirme

### Yeni Özellik Ekleme

1. Yeni node oluştur
2. Mesaj tiplerini tanımla
3. Launch dosyasını güncelle
4. Test et

### Debug

```bash
# Topic'leri izle
ros2 topic echo /air_defense/command
ros2 topic echo /aircraft/status

# Node durumunu kontrol et
ros2 node list
ros2 node info /aircraft_controller

# Log'ları izle
ros2 run air_defense_system aircraft_controller.py --ros-args --log-level debug
```

## Katkıda Bulunma

1. Fork yap
2. Feature branch oluştur
3. Değişiklikleri commit et
4. Pull request gönder

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır.

## İletişim

- Takım: Hava Savunma Takımı
- Email: team@example.com
- GitHub: [repository_url]
