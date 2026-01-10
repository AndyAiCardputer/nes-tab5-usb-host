# 🚀 Quick Start - Web Flasher

## ✅ What's Already Done

- ✅ All files created and committed
- ✅ Working firmware from `xxx1` copied
- ✅ Manifest configured for merged file (simpler and more reliable)
- ✅ GitHub Pages configured via branch

## 📋 What to Do Now

### 1. Push Changes

```bash
cd /Users/a15/A_AI_Project/cardputer/tab5/tab5_nes_emul_volume
git push
```

### 2. Wait for GitHub Pages Publication

After pushing, GitHub will automatically publish the page within 1-2 minutes.

### 3. Open Web Flasher

The page will be available at:
```
https://andyaiCardputer.github.io/[repository-name]/webflasher/
```

For example, if the repository is named `nes-tab5-usb-host`:
```
https://andyaiCardputer.github.io/nes-tab5-usb-host/webflasher/
```

### 4. Flash Tab5

1. Open the page in browser (Chrome/Edge)
2. Connect Tab5 via USB
3. Click the **"Install"** button on the page
4. Grant serial port access (browser will ask)
5. Select device port (usually `/dev/cu.usbmodemXXXX`)
6. Wait for flashing to complete (will show progress)
7. Tab5 will automatically reboot

## 📁 File Structure

```
webflasher/
├── index.html                    # Web page
├── manifest.json                 # Uses merged file (recommended)
├── manifest-three-files.json     # Alternative (three files)
└── firmware/
    ├── tab5_nes_merged.bin       # ⭐ Main file (712 KB)
    ├── bootloader.bin            # (24 KB) - for alternative option
    ├── partition-table.bin        # (3 KB) - for alternative option
    └── nes_tab5_file_browser.bin # (648 KB) - for alternative option
```

## ⚙️ Using Merged File vs Three Files

### Merged File (Recommended) ✅
- **Simpler**: one file instead of three
- **More reliable**: fewer chances for errors
- **Faster**: single flash operation
- Used in `manifest.json` by default

### Three Files (Alternative)
- If you need to update only one component
- Used in `manifest-three-files.json`
- In `index.html` change `manifest="manifest.json"` to `manifest="manifest-three-files.json"`

## 🔧 Updating Firmware

When you build a new version:

1. Copy new merged file:
   ```bash
   cp build/tab5_nes_merged.bin webflasher/firmware/
   ```

2. Or copy three files:
   ```bash
   cp build/bootloader/bootloader.bin webflasher/firmware/
   cp build/partition_table/partition-table.bin webflasher/firmware/
   cp build/nes_tab5_file_browser.bin webflasher/firmware/
   ```

3. Commit and push:
   ```bash
   git add webflasher/firmware/
   git commit -m "feat: update firmware to vX.X.X"
   git push
   ```

4. GitHub Pages will automatically update!

## 🌐 Browsers

Browsers with Web Serial API support:
- ✅ Chrome 89+
- ✅ Edge 89+
- ✅ Opera 75+
- ❌ Firefox (does not support Web Serial API)
- ❌ Safari (does not support Web Serial API)

## 🐛 Troubleshooting

### Page Won't Open
- Check that GitHub Pages is published (Settings → Pages)
- Wait 1-2 minutes after pushing
- Check URL (should have `/webflasher/` at the end)

### Device Not Found
- Press Reset button on Tab5
- Close Serial Monitor or other programs
- Use a data USB cable (not charging-only)

### Flashing Not Working
- Open browser console (F12) and check for errors
- Verify that manifest.json is accessible via HTTP
- Ensure browser supports Web Serial API

### GitHub Pages Not Updating
- Check that files are committed and pushed
- Wait 1-2 minutes
- Check logs in Settings → Pages → View deployment logs

---

**Ready!** After pushing, the web flasher will work! 🎮
