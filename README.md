# NES Emulator for M5Stack Tab5 with USB Host Support

NES emulator for M5Stack Tab5 based on ESP-IDF framework with USB Host support for gamepad connectivity.

## ✅ Current Status

- ✅ Basic ESP-IDF project structure created
- ✅ `nes_emulator` component with nofrendo library
- ✅ Basic OSD functions for ESP-IDF
- ✅ Tab5 display initialization (ST7123)
- ✅ USB Host support for gamepads
- ✅ USB gamepad integration into NES emulator input system
- ⚠️ Display scaling (placeholder)
- ⚠️ ES8388 audio (placeholder)
- ⚠️ nofrendo main loop integration (TODO)

## 📁 Project Structure

```
nes_tab5_usb_host/
├── CMakeLists.txt              # Root CMakeLists
├── sdkconfig.defaults          # ESP-IDF configuration
├── partitions.csv               # Partition table
├── main/
│   ├── CMakeLists.txt          # CMakeLists for main
│   ├── idf_component.yml       # Dependencies (USB Host HID)
│   └── app_main.cpp            # Main application file
└── components/
    ├── m5stack_tab5/           # BSP component
    └── nes_emulator/           # NES emulator component
        ├── CMakeLists.txt
        ├── idf_component.yml
        ├── include/
        │   └── nes_osd.h       # OSD header
        └── src/
            └── nes_osd.c       # OSD implementation (with USB Host)
```

## 🎮 USB Gamepad Support

### Supported Gamepads

- **PlayStation 5 DualSense** (VID: 0x054C, PID: 0x0CE6) - Full support
- **Generic USB HID gamepads** - Basic support

### Button Mapping to NES

**PS5 DualSense:**
- **D-Pad** → NES D-Pad (Up/Down/Left/Right)
- **Left Stick** → NES D-Pad (with ±40 threshold)
- **Cross (X)** → NES B
- **Circle (O)** → NES A
- **Square** → Turbo B (rapid fire at 10 Hz)
- **Triangle** → Turbo A (rapid fire at 10 Hz)
- **Create** → NES Select
- **Options** → NES Start

**Generic USB HID:**
- D-Pad and buttons are mapped according to standard USB HID format

## 🔧 Compilation

### Requirements

- ESP-IDF v5.4 or higher
- ESP32-P4 toolchain

### Build and Flash

```bash
cd /Users/a15/A_AI_Project/cardputer/tab5/nes_tab5_usb_host

# Set up ESP-IDF environment (if not already set)
export IDF_PATH=/Users/a15/A_AI_Project/esp-idf-official
source $IDF_PATH/export.sh

# Configure project
idf.py set-target esp32p4

# Build project
idf.py build

# Flash to Tab5
idf.py -p /dev/cu.usbmodem1434301 flash

# Open monitor
idf.py -p /dev/cu.usbmodem1434301 monitor
```

## 📝 Usage

1. Copy NES game ROM file to `/sd/roms/game.nes` on SD card
2. Insert SD card into Tab5
3. Connect USB gamepad to USB-A port on Tab5
4. Flash firmware and start
5. Emulator will automatically detect gamepad and start emulation

## 📝 TODO

- [ ] Implement scaling 256×240 → 1280×720 in `nes_display_render_frame()`
- [ ] Integrate nofrendo main loop into `app_main.cpp`
- [ ] Implement ES8388 audio codec
- [ ] Add ROM loading from SD card
- [ ] Optimize rendering (possibly DMA)
- [ ] Add Nintendo Switch Pro Controller support
- [ ] Add Xbox gamepad support

## 🔗 Links

- **Nofrendo:** https://github.com/imhof/nofrendo
- **ESP-IDF:** https://docs.espressif.com/projects/esp-idf/
- **USB Host Guide:** See `USB_HOST_GUIDE.md` for detailed USB Host and gamepad integration guide
- **ZX Spectrum Tab5 (reference):** `cardputer/tab5/zx_spectrum_tab5_idf/`

## 📝 Changelog

### v1.0.0 (2025-01-XX)
- Initial ESP-IDF port from Arduino framework
- Basic project structure
- Display initialization
- USB Host gamepad support integrated
- Gamepad input mapping to NES buttons
- Turbo button support (Square/Triangle for rapid fire)
