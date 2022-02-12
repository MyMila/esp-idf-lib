# What the example does

The example sets (hard-coded) date and time. In the loop, it reads temperature
and time from the device, prints them to the serial console.

```console
I (27) boot: ESP-IDF v4.2-dev-3053-g74d6192ef-dirty 2nd stage bootloader
I (27) boot: compile time 21:39:36
I (29) boot: chip revision: 3
I (32) boot_comm: chip revision: 3, min. bootloader chip revision: 0
I (39) boot.esp32: SPI Speed      : 40MHz
I (44) boot.esp32: SPI Mode       : DIO
I (48) boot.esp32: SPI Flash Size : 2MB
I (53) boot: Enabling RNG early entropy source...
I (58) boot: Partition Table:
I (62) boot: ## Label            Usage          Type ST Offset   Length
I (69) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (77) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (84) boot:  2 factory          factory app      00 00 00010000 00100000
I (92) boot: End of partition table
I (96) boot_comm: chip revision: 3, min. application chip revision: 0
I (103) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x07390 ( 29584) map
I (123) esp_image: segment 1: paddr=0x000173b8 vaddr=0x3ffb0000 size=0x020bc (  8380) load
I (127) esp_image: segment 2: paddr=0x0001947c vaddr=0x40080000 size=0x06b9c ( 27548) load
0x40080000: _WindowOverflow4 at /Users/msia3/esp/esp-idf/components/freertos/xtensa/xtensa_vectors.S:1730

I (144) esp_image: segment 3: paddr=0x00020020 vaddr=0x400d0020 size=0x16540 ( 91456) map
0x400d0020: _stext at ??:?

I (179) esp_image: segment 4: paddr=0x00036568 vaddr=0x40086b9c size=0x047c0 ( 18368) load
0x40086b9c: xTaskRemoveFromEventList at /Users/msia3/esp/esp-idf/components/freertos/tasks.c:3115

I (194) boot: Loaded app from partition at offset 0x10000
I (194) boot: Disabling RNG early entropy source...
I (194) cpu_start: Pro cpu up.
I (198) cpu_start: Application information:
I (203) cpu_start: Project name:     example-ens210
I (208) cpu_start: App version:      0.8.2-30-g255434e-dirty
I (215) cpu_start: Compile time:     Jan  9 2022 21:48:47
I (221) cpu_start: ELF file SHA256:  efe31873014f89c9...
I (227) cpu_start: ESP-IDF:          v4.2-dev-3053-g74d6192ef-dirty
I (234) cpu_start: Starting app cpu, entry point is 0x40081668
0x40081668: call_start_cpu1 at /Users/msia3/esp/esp-idf/components/esp32/cpu_start.c:287

I (0) cpu_start: App cpu up.
I (244) heap_init: Initializing. RAM available for dynamic allocation:
I (251) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (257) heap_init: At 3FFB2930 len 0002D6D0 (181 KiB): DRAM
I (263) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (270) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (276) heap_init: At 4008B35C len 00014CA4 (83 KiB): IRAM
I (282) cpu_start: Pro cpu start user code
I (300) spi_flash: detected chip: gd
I (301) spi_flash: flash io: dio
W (301) spi_flash: Detected size(8192k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (311) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (140) ENS210: Temperature: 27.97 C, Humidity: 43.23 %RH
I (1270) ENS210: Temperature: 27.97 C, Humidity: 43.15 %RH
I (2400) ENS210: Temperature: 27.98 C, Humidity: 43.00 %RH
I (3530) ENS210: Temperature: 27.97 C, Humidity: 42.80 %RH
I (4660) ENS210: Temperature: 27.95 C, Humidity: 42.57 %RH
I (5790) ENS210: Temperature: 27.97 C, Humidity: 42.44 %RH
...
```

## Connecting

| `ENS210` | `ESP8266` | `ESP32`  | `NodeMCU` |
|----------|-----------|----------|-----------|
| `SCL`    | `GPIO5`   | `GPIO26` | `D1`      |
| `SDA`    | `GPIO4`   | `GPIO25` | `D2`      |

