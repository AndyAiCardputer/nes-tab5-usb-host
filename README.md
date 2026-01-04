# NES Emulator for M5Stack Tab5 with File Browser

NES emulator for M5Stack Tab5 with built-in file browser for selecting ROM files.

## Features

- **File Browser**: Browse and select NES ROM files from SD card
- **USB Host Support**: Use USB gamepad (PS5 DualSense only.) for navigation
- **I2C Input Support**: Joystick2, Scroll buttons, CardKeyBoard
- **Display**: 1280×720 landscape orientation
- **Audio**: ES8388 codec support

## Hardware Requirements

- M5Stack Tab5 (ESP32-P4)
- SD Card (FAT32) with ROM files in `/sd/roms/` folder
- USB gamepad (optional, for navigation)

## ROM Files

Place your `.nes` ROM files in `/sd/roms/` folder on the SD card.

Example:
```
/sd/roms/
  ├── super_mario.nes
  ├── zelda.nes
  └── metroid.nes
```

## Building

```bash
cd nes_tab5_usb_host
export IDF_PATH=/path/to/esp-idf
source $IDF_PATH/export.sh
idf.py build
```

## Flashing

### Option 1: Flash from Source (Recommended)

```bash
cd nes_tab5_usb_host
export IDF_PATH=/path/to/esp-idf
source $IDF_PATH/export.sh
idf.py -p /dev/cu.usbmodemXXXX flash
```

### Option 2: Flash from GitHub Release

Download all three files from the release:
- `bootloader.bin`
- `partition-table.bin`
- `nes_tab5_file_browser.bin`

Then flash them using esptool:

```bash
# Make sure ESP-IDF environment is loaded
export IDF_PATH=/path/to/esp-idf
source $IDF_PATH/export.sh

# Flash all three files
esptool --chip esp32p4 --port /dev/cu.usbmodemXXXX --baud 921600 write_flash \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 nes_tab5_file_browser.bin
```

**Important:** All three files are required! Flashing only the main binary will result in "invalid header" errors.

## Usage

1. Insert SD card with ROM files in `/sd/roms/` folder
2. Connect USB gamepad (optional)
3. Power on Tab5
4. File browser will appear showing available ROM files
5. Navigate with D-Pad Up/Down
6. Press Start or A to launch selected game
7. To return to file browser, restart Tab5

## Controls

### File Browser
- **D-Pad Up/Down**: Navigate file list
- **Start or A**: Launch selected game

### In-Game (PS5 DualSense)
- **Cross (PS5)**: NES B
- **Circle (PS5)**: NES A
- **Square**: Turbo B
- **Triangle**: Turbo A
- **Create**: Select
- **Options**: Start
- **D-Pad / Left Stick**: D-Pad

## Project Structure

```
nes_tab5_usb_host/
├── main/
│   └── app_main.cpp          # Main application with file browser
├── components/
│   ├── m5stack_tab5/         # BSP for Tab5
│   └── nes_emulator/         # NES emulator (nofrendo)
│       ├── include/
│       │   └── nes_osd.h     # OSD functions
│       └── src/
│           └── nes_osd.c     # OSD implementation
├── CMakeLists.txt
├── sdkconfig.defaults
└── partitions.csv
```

## License

See component licenses.

