# 🚀 Web Flasher Setup - Quick Start Guide

## What Was Created

I've created a web-based flasher for the Tab5 NES Emulator that allows you to flash the device directly from your browser without installing drivers or software!

### File Structure:

```
webflasher/
├── index.html              # Web page with flasher interface
├── manifest.json          # Firmware description (addresses, files)
├── README.md              # Documentation
├── SETUP.md               # Detailed setup instructions
└── firmware/              # Firmware binary files
    ├── bootloader.bin
    ├── partition-table.bin
    └── nes_tab5_file_browser.bin

.github/
└── workflows/
    └── deploy-pages.yml   # GitHub Actions for automatic deployment
```

## How It Works

1. **esp-web-tools** - Espressif library that uses browser's Web Serial API
2. **Web Serial API** - Allows browser to directly communicate with USB devices
3. **GitHub Pages** - Hosting for the web page
4. **GitHub Actions** - Automatic deployment when files are updated

## What to Do Next

### Step 1: Configure GitHub Pages

1. Go to your repository on GitHub
2. Navigate to **Settings** → **Pages**
3. In the **Source** section, select **"Deploy from a branch"** (classic method)
4. Select branch: **main** (or **master**)
5. Select folder: **/ (root)** or **/webflasher** (if you want the page in a subfolder)
6. Click **Save**

### Step 2: Commit and Push Files

```bash
cd /Users/a15/A_AI_Project/cardputer/tab5/tab5_nes_emul_volume

# Check that all files are in place
git status

# Add new files
git add webflasher/ .github/workflows/deploy-pages.yml

# Commit
git commit -m "feat: add web flasher for Tab5 NES emulator"

# Push
git push
```

### Step 3: Wait for Deployment

1. Go to **Actions** on GitHub
2. Wait for workflow **Deploy Web Flasher to GitHub Pages** to complete
3. After successful deployment, the page will be available at:
   ```
   https://[your-username].github.io/[repository-name]/webflasher/
   ```

### Step 4: Test

1. Open the web flasher page in browser (Chrome/Edge)
2. Connect Tab5 via USB
3. Click the "Install" button on the page
4. Grant serial port access
5. Select device port
6. Wait for flashing to complete

## Updating Firmware

When you build a new firmware version:

1. Copy new binary files:
   ```bash
   cp build/bootloader/bootloader.bin release_binaries/
   cp build/partition_table/partition-table.bin release_binaries/
   cp build/nes_tab5_file_browser.bin release_binaries/
   ```

2. Copy to webflasher:
   ```bash
   cp release_binaries/*.bin webflasher/firmware/
   ```

3. Commit and push - GitHub Pages will automatically update the web flasher!

## Technical Details

### Flash Addresses (from manifest.json):

- **0x0** (0) - Bootloader
- **0x8000** (32768) - Partition table  
- **0x10000** (65536) - Main application

### Supported Browsers:

- Chrome 89+
- Edge 89+
- Opera 75+
- Other browsers with Web Serial API support

### File Sizes:

- bootloader.bin: ~24 KB
- partition-table.bin: ~3 KB
- nes_tab5_file_browser.bin: ~641 KB

## Web Flasher Advantages

✅ **No drivers needed** - works through browser  
✅ **Cross-platform** - Windows, macOS, Linux  
✅ **Simple** - one click and done  
✅ **Automatic updates** - when pushing to GitHub  
✅ **Accessible** - can share link with others  

## Troubleshooting

### GitHub Pages Not Updating
- Check logs in **Actions**
- Ensure files are committed
- Verify GitHub Pages is set to use branch as source

### Flashing Not Working in Browser
- Open browser console (F12) and check for errors
- Ensure browser supports Web Serial API
- Verify that manifest.json is accessible via HTTP

### Device Not Found
- Press Reset button on Tab5
- Close Serial Monitor or other programs using the port
- Use a data USB cable (not charging-only)

## Additional Information

- Detailed documentation: `webflasher/README.md`
- Setup instructions: `webflasher/SETUP.md`
- ESP Web Tools: https://github.com/espressif/esp-web-tools

---

**Ready!** Now you have a web flasher for the NES emulator! 🎮
