# Hava savunma komutları
Header header

# Komut tipi
uint8 COMMAND_ACTIVATE = 1
uint8 COMMAND_DEACTIVATE = 2
uint8 COMMAND_UPDATE_AREAS = 3
uint8 COMMAND_EMERGENCY_LAND = 4

uint8 command_type

# Komut parametreleri
string[] parameters

# Komut önceliği (0: Düşük, 1: Normal, 2: Yüksek, 3: Acil)
uint8 priority

# Komut zaman damgası
time timestamp
