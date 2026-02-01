# ATF and u-boot for mt798x with DHCPD

A modified version of hanwckf's U-Boot for MT798x by Yuzhii, with support for DHCPD and a beautiful web UI.

Supports GitHub Actions for automatic builds, and can generate both normal and overclocked BL2.

**Warnign: Flashing custom bootloaders can brick your device. Proceed with caution and at your own risk.**

**Version 2026 is in beta, please report any issues you encounter.**

## About bl-mt798x

> Version-2026 WEB UI preview

U-Boot 2026 adds more features:

- System info display
- Factory (RF) update
- Backup download
- Web terminal
- Environment manager
- Device reboot

![Version-2026](document/pictures/uboot-2025.png)

You can configure the features you need.

- [x] MTK_DHCPD
  - [x] MTK_DHCPD_ENHANCED
- Failsafe Web UI style:
  - [x] WEBUI_FAILSAFE_UI_NEW
  - [ ] WEBUI_FAILSAFE_UI_OLD
- [x] WEBUI_FAILSAFE_FACTORY
- [x] WEBUI_FAILSAFE_BACKUP
- [x] WEBUI_FAILSAFE_ENV
- [x] WEBUI_FAILSAFE_CONSOLE

> Enable WEBUI_FAILSAFE_UI_OLD will use the traditional webui interface.

## Prepare

```bash
sudo apt install gcc-aarch64-linux-gnu build-essential flex bison libssl-dev device-tree-compiler qemu-user-static
```

## Build

```bash
chmod +x build.sh
SOC= BOARD= ./build.sh
```

- SOC=mt7981/mt7986
- MULTI_LAYOUT=0

> MULTI_LAYOUT default=1, set to 0 to disable multi layout, if board is not support multi layout will fallback to single layout automatically.

| Version | ATF | UBOOT |
| --- | --- | --- |
| 2026 | 20260123 | 20260123 |

Generated files will be in the `output`
