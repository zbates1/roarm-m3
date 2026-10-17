# Firmware

## Backup of the arm as shipped (2026-10-17)

| | |
|---|---|
| File | `firmware/backup/roarm_m3_flash_16MB_2026-10-17.bin` (git-ignored) and a copy in `D:\roarm-m3-backups\` |
| SHA-256 | `1da3aadccd4c7641c0cbe835a0aeefb28d636a784f26734db6adf66b5383d04b` |
| Chip | ESP32-D0WD-V3 rev 3.1, MAC `ec:c9:ff:f1:3f:64`, **16 MB** flash |
| Partitions | standard 4 MB layout (rest of the 16 MB unused): nvs 0x9000, otadata 0xE000, app0 0x10000 (1280 KB, active), app1 0x150000 (empty), spiffs/LittleFS 0x290000 (1408 KB), coredump 0x3F0000 |
| Program | `app0` is **byte-for-byte identical** to Waveshare's prebuilt `RoArm-M3_example.ino.bin` (RoArm-M3_example_20260701.zip) |

**The backup contains the WiFi password** (in the LittleFS `boot.mission` and `wifiConfig.json`). Never commit it or share it.

Restore everything exactly as it was:

```powershell
$esptool = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.0\esptool.exe"
& $esptool --chip esp32 -p COM14 -b 921600 write-flash 0 firmware\backup\roarm_m3_flash_16MB_2026-10-17.bin
```

## Building Waveshare's source

Waveshare's build map shows what they compiled with: Arduino-ESP32 core 3.0.x (ESP-IDF 5.1), ArduinoJson 7.3.1, ESPAsyncWebServer (master) + AsyncTCP, Adafruit SSD1306 + GFX, INA219_WE, SCServo.
