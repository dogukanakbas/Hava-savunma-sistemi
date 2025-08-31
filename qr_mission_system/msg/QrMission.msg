# QR Kod Görevi Mesajı
Header header

# QR kod konumu (GPS)
float64 enlem
float64 boylam
float64 irtifa

# Dalış parametreleri
int32 min_yukseklik  # Minimum dalış yüksekliği (metre)
int32 baslangic_yuksekligi  # Başlangıç yüksekliği (metre)
float32 dalis_acisi  # Dalış açısı (derece)

# Görev durumu
bool is_active  # Görevi başlat/durdur
bool emergency_stop  # Acil durdurma
