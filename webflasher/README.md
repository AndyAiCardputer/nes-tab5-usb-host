# Web Flasher for NES Emulator Tab5

This directory contains the web-based flasher for the NES Emulator on M5Stack Tab5.

## How It Works

The web flasher uses [esp-web-tools](https://github.com/espressif/esp-web-tools) to flash firmware directly from your browser using Web Serial API. No drivers or software installation needed!

## Files

- `index.html` - Main web page with flasher interface
- `manifest.json` - Firmware manifest describing flash addresses
- `firmware/` - Directory containing binary files:
  - `bootloader.bin` - ESP32-P4 bootloader (address 0x0)
  - `partition-table.bin` - Partition table (address 0x8000)
  - `nes_tab5_file_browser.bin` - Main application (address 0x10000)

## Flash Addresses

- **0x0** (0) - Bootloader
- **0x8000** (32768) - Partition table
- **0x10000** (65536) - Main application

## Deployment

The web flasher is automatically deployed to GitHub Pages when:
- Files in `webflasher/` directory are updated
- Files in `release_binaries/` directory are updated
- GitHub Actions workflow is manually triggered

## Usage

1. Open the GitHub Pages URL (e.g., `https://andyaiCardputer.github.io/repo-name/webflasher/`)
2. Connect Tab5 via USB
3. Click the "Install" button
4. Grant serial port access
5. Select your device port
6. Wait for flashing to complete

## Browser Support

- Chrome 89+
- Edge 89+
- Opera 75+
- Other browsers with Web Serial API support

## Troubleshooting

- **Device not found**: Press Reset button on Tab5
- **Port busy**: Close Serial Monitor or other applications
- **Flashing fails**: Use a data USB cable (not charging-only)
- **Browser not supported**: Update to latest Chrome/Edge version
