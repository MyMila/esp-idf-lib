# What the example does

The example sets (hard-coded) date and time. In the loop, it reads temperature
and time from the device, prints them to the serial console.

```I (29) boot: ESP-IDF v4.4 2nd stage bootloader
I (29) boot: compile time 20:20:07
I (29) boot: chip revision: 3
I (32) boot_comm: chip revision: 3, min. bootloader chip revision: 0
I (39) boot.esp32: SPI Speed      : 40MHz
I (43) boot.esp32: SPI Mode       : DIO
I (48) boot.esp32: SPI Flash Size : 2MB
I (53) boot: Enabling RNG early entropy source...
I (58) boot: Partition Table:
I (62) boot: ## Label            Usage          Type ST Offset   Length
I (69) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (76) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (84) boot:  2 factory          factory app      00 00 00010000 00100000
I (91) boot: End of partition table
I (95) boot_comm: chip revision: 3, min. application chip revision: 0
I (103) esp_image: segment 0: paddr=00010020 vaddr=3f400020 size=0975ch ( 38748) map
I (125) esp_image: segment 1: paddr=00019784 vaddr=3ffb0000 size=023a0h (  9120) load
I (129) esp_image: segment 2: paddr=0001bb2c vaddr=40080000 size=044ech ( 17644) load
I (138) esp_image: segment 3: paddr=00020020 vaddr=400d0020 size=17ec4h ( 97988) map
I (175) esp_image: segment 4: paddr=00037eec vaddr=400844ec size=083e4h ( 33764) load
I (189) esp_image: segment 5: paddr=000402d8 vaddr=50000000 size=00010h (    16) load
I (195) boot: Loaded app from partition at offset 0x10000
I (196) boot: Disabling RNG early entropy source...
I (209) cpu_start: Pro cpu up.
I (209) cpu_start: Starting app cpu, entry point is 0x400810ec
0x400810ec: call_start_cpu1 at /Users/msia3/esp/esp-idf-v4.4/components/esp_system/port/cpu_start.c:156

I (0) cpu_start: App cpu up.
I (223) cpu_start: Pro cpu start user code
I (223) cpu_start: cpu freq: 160000000
I (224) cpu_start: Application information:
I (228) cpu_start: Project name:     example-ens160
I (234) cpu_start: App version:      0.8.2-58-gf0705c2-dirty
I (240) cpu_start: Compile time:     Feb 14 2022 20:20:25
I (246) cpu_start: ELF file SHA256:  53a09a37f110fb8f...
I (252) cpu_start: ESP-IDF:          v4.4
I (257) heap_init: Initializing. RAM available for dynamic allocation:
I (264) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (270) heap_init: At 3FFB2CE0 len 0002D320 (180 KiB): DRAM
I (276) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (283) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (289) heap_init: At 4008C8D0 len 00013730 (77 KiB): IRAM
I (296) spi_flash: detected chip: gd
I (300) spi_flash: flash io: dio
W (303) spi_flash: Detected size(8192k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (318) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (60) ENS160: ======================
I (60) ENS160: AQI: 0
I (60) ENS160: TVOC: 0 ppb
I (60) ENS160: eCO2: 0 ppm
I (60) ENS160: NO2: 0 ppb
I (1060) ENS160: ======================
I (1060) ENS160: AQI: 0
I (1060) ENS160: TVOC: 0 ppb
I (1060) ENS160: eCO2: 0 ppm
I (1060) ENS160: NO2: 0 ppb
I (2060) ENS160: ======================
I (2060) ENS160: AQI: 1
I (2060) ENS160: TVOC: 24 ppb
I (2060) ENS160: eCO2: 400 ppm
I (2060) ENS160: NO2: 24 ppb
I (3060) ENS160: ======================
I (3060) ENS160: AQI: 1
I (3060) ENS160: TVOC: 21 ppb
I (3060) ENS160: eCO2: 400 ppm
I (3060) ENS160: NO2: 21 ppb
...
```

## Connecting

| `ENS210` | `ESP8266` | `ESP32`  | `NodeMCU` |
|----------|-----------|----------|-----------|
| `SCL`    | `GPIO5`   | `GPIO26` | `D1`      |
| `SDA`    | `GPIO4`   | `GPIO25` | `D2`      |

