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

## Building

`RoArm-M3_example/` is Waveshare's source (commit 0254eb2 is the unmodified original) plus our additions in `servo_regs.h`:

| Command | Does |
|---|---|
| `{"T":116,"id":12,"addr":22,"len":1}` | read any servo register → `{"T":1161,...,"val":32,"ok":1}` |
| `{"T":117,"id":12,"addr":22,"len":1,"val":40}` | write P 21 / D 22 / I 23 / min startup force 24 (2 bytes) / dead zones 26, 27 / acceleration 41; anything else is refused. Never unlocks EEPROM. Reads the value back. |

In Python: `arm.read_reg(id, addr, length)` and `arm.write_reg(id, addr, value, length)`.

Toolchain (all on D:, C: is nearly full), matching Waveshare's build map:
- `D:\arduino\arduino-cli\arduino-cli.exe` with config `D:\arduino\arduino-cli.yaml` (data, downloads, libraries and build cache under `D:\arduino`)
- Arduino-ESP32 core **3.0.7** (their binary contains "3.0.7" and IDF v5.1-632e0c2a)
- ArduinoJson 7.3.1, ESP Async WebServer 3.7.9 (same source files as their "master" build) + Async TCP 3.4.4, Adafruit SSD1306 2.5.17 + GFX 1.12.6, **INA219_WE 1.3.8** (1.4.x renamed constants), SCServo from `waveshareteam/ugv_base_general` (commit d308df9)

```bash
export TMP=D:/arduino/tmp TEMP=D:/arduino/tmp
CLI="/d/arduino/arduino-cli/arduino-cli.exe --config-file D:/arduino/arduino-cli.yaml"
$CLI compile --fqbn esp32:esp32:esp32 --build-path D:/arduino/build/roarm_regs firmware/RoArm-M3_example
# flash only the program slot (keeps bootloader, partitions, WiFi settings, boot.mission):
esptool --chip esp32 -p COM14 -b 921600 write-flash 0x10000 D:/arduino/build/roarm_regs/RoArm-M3_example.ino.bin
```

The program is 98 % of the 1280 KB slot, so additions have ~20 KB to spare.

## Servo registers as read on 2026-10-17 (all seven ST3215 servos, model 777 = STS3215, firmware 3.9)

P 16 (set by the firmware at boot), **D 32**, I 0, minimum startup force 16 (gripper 0), dead zones 1/1, max torque 1000, protection current 310 (gripper 230), protective torque 20 %, protection time 200, overload torque 80 % (gripper 50), velocity loop P 10 / I 200 (gripper 10), EEPROM locked (55 = 1). Register 81 ("DTs", ms) = 20 (gripper 10); its meaning isn't documented, so it's left alone.
