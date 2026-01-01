# Flashing Instructions for NES Emulator Tab5

## Files Required

All three binary files are required for successful flashing:

1. **bootloader.bin** (24 KB) - Bootloader for ESP32-P4
2. **partition-table.bin** (3 KB) - Partition table configuration
3. **nes_tab5_file_browser.bin** (641 KB) - Main application

## Flashing Steps

### 1. Connect Tab5 via USB

Connect your M5Stack Tab5 to your computer via USB cable.

### 2. Find the USB Port

On macOS/Linux:
```bash
ls -la /dev/cu.usbmodem*
```

On Windows, check Device Manager for COM port.

### 3. Flash All Three Files

Make sure ESP-IDF environment is loaded:

```bash
export IDF_PATH=/path/to/esp-idf
source $IDF_PATH/export.sh
```

Then flash all files (replace `/dev/cu.usbmodemXXXX` with your actual port):

```bash
esptool --chip esp32p4 --port /dev/cu.usbmodemXXXX --baud 921600 write_flash \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 nes_tab5_file_browser.bin
```

### 4. Verify Installation

After flashing, monitor the serial output:

```bash
idf.py -p /dev/cu.usbmodemXXXX monitor
```

You should see initialization messages and the file browser if SD card is inserted.

## Troubleshooting

### Error: "invalid header: 0xffffffff"

This means bootloader or partition table is missing. Make sure you flash **all three files** in the correct order.

### Error: "Device not found"

- Check USB cable connection
- Try pressing Reset button on Tab5
- Check if port changed (run `ls -la /dev/cu.usbmodem*` again)

### Black Screen After Flashing

- Check serial monitor for error messages
- Ensure SD card is inserted with ROM files in `/sd/roms/` folder
- Try erasing flash first: `esptool --chip esp32p4 --port /dev/cu.usbmodemXXXX erase_flash`

## File Addresses

- **0x0** - Bootloader
- **0x8000** - Partition table
- **0x10000** - Main application (factory partition)

These addresses are fixed and must not be changed.

