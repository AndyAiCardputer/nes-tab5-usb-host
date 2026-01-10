# Setup Guide for Web Flasher

This guide explains how to set up and deploy the web flasher to GitHub Pages.

## Prerequisites

- GitHub repository for the NES Emulator project
- GitHub Actions enabled (enabled by default)

## Step 1: Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** → **Pages**
3. Under **Source**, select **GitHub Actions**
4. Save the settings

## Step 2: Push Files to Repository

The following files should be committed and pushed:

```
webflasher/
├── index.html
├── manifest.json
├── README.md
└── firmware/
    ├── bootloader.bin
    ├── partition-table.bin
    └── nes_tab5_file_browser.bin

.github/
└── workflows/
    └── deploy-pages.yml
```

## Step 3: Update Binary Files

When you update the firmware binaries:

1. Build new binaries using `idf.py build`
2. Copy new binaries to `release_binaries/`:
   ```bash
   cp build/bootloader/bootloader.bin release_binaries/
   cp build/partition_table/partition-table.bin release_binaries/
   cp build/nes_tab5_file_browser.bin release_binaries/
   ```
3. Copy binaries to `webflasher/firmware/`:
   ```bash
   cp release_binaries/*.bin webflasher/firmware/
   ```
4. Commit and push changes
5. GitHub Actions will automatically deploy to GitHub Pages

## Step 4: Access Web Flasher

After deployment, your web flasher will be available at:

```
https://[your-username].github.io/[repository-name]/webflasher/
```

For example:
```
https://andyaiCardputer.github.io/tab5-nes-emulator/webflasher/
```

## Manual Deployment

You can also trigger deployment manually:

1. Go to **Actions** tab in GitHub
2. Select **Deploy Web Flasher to GitHub Pages** workflow
3. Click **Run workflow**
4. Select branch (usually `main` or `master`)
5. Click **Run workflow**

## Troubleshooting

### GitHub Pages Not Updating

- Check GitHub Actions logs for errors
- Ensure `webflasher/` files are committed
- Verify GitHub Pages is set to use GitHub Actions source

### Binary Files Too Large

GitHub has a 100MB file size limit. If binaries are too large:
- Use Git LFS (Large File Storage) for binary files
- Or host binaries on a CDN and update manifest.json URLs

### Web Flasher Not Working

- Check browser console for errors (F12)
- Ensure browser supports Web Serial API
- Verify manifest.json paths are correct
- Check that firmware files are accessible via HTTP

## File Structure Explanation

- **index.html**: Main web page with esp-web-tools integration
- **manifest.json**: Describes firmware structure and flash addresses
- **firmware/**: Contains binary files for flashing
- **.github/workflows/deploy-pages.yml**: GitHub Actions workflow for deployment

## Flash Addresses

The manifest.json uses decimal offsets:
- Bootloader: `0` (0x0)
- Partition table: `32768` (0x8000)
- Application: `65536` (0x10000)

These match the ESP32-P4 flash layout.
