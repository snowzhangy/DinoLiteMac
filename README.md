# DinoLiteMac

A modern macOS (Apple Silicon) viewer for the **Dino-Lite AM411T** USB microscope,
whose vendor driver (DinoXcope) no longer works on current macOS. It talks to the
device directly over `libusb` and renders a live, full-resolution image with Metal.

The AM411T is a **Sonix SN9C201** USB bridge + **Micron/Aptina MT9M111** 1.3 MP SOC
sensor (USB `a168:0615`). The interface is vendor-specific (class `0xFF`), so macOS
won't bind it as a UVC camera. The control protocol here is ported from the Linux
kernel `gspca sn9c20x` driver.

## Features

- Live **1280×1023 SXGA** preview, GPU-accelerated (Metal).
- Correct color: GPU debayer (BGGR, block chroma + full-res luma), white balance,
  **sRGB gamma**, and a contrast control.
- **Hardware auto-exposure** (the MT9M111 runs its own AE/AGC — stable, no pumping).
- **Auto white balance** (white-patch; converges then locks, no flicker).
- **PNG capture** to the Desktop — via the on-screen button or the **Spacebar**.
- LED on/off.
- Runtime **Bayer phase** selector (in case a different unit ships a different phase).

## Requirements

- Apple Silicon Mac, recent macOS.
- [`libusb`](https://libusb.info/): `brew install libusb`

## Build & run

```sh
make          # builds dino_metal (GUI) and dino_grab (CLI)
./dino_metal  # or: make run
```

> Plug in the microscope first. If the app can't claim the device, unplug/replug it.

## Controls (GUI)

| Control      | Action                                                        |
|--------------|---------------------------------------------------------------|
| Capture / ␣  | Save a PNG to `~/Desktop`                                      |
| LED On/Off   | Toggle the ring light (single GPIO LED — no brightness levels) |
| Gray         | Show raw luminance (no debayer) — best for fine detail        |
| Lock WB      | Re-run auto white balance, then lock                          |
| Bayer        | Cycle the Bayer phase (default **BGGR**) if colors look wrong  |
| Contrast     | sRGB-space contrast around mid-gray                           |

## CLI grabber

`dino_grab` streams the same RAW Bayer without a GUI:

```sh
./dino_grab                     # save 3 PPM frames (+ raw PGM) to /tmp
./dino_grab - | ffplay -f rawvideo -pixel_format rgb24 -video_size 1280x1023 -i -
```

## Notes / quirks discovered

- **SXGA is RAW-only.** The bridge's hardware YUV/JPEG (with on-chip demosaic + color)
  is limited to ≤ 640×480, so full resolution must be debayered in software/GPU.
- The sensor emits **1023** rows at SXGA (one short of 1024); the window registers
  still use 1024.
- The **physical touch button** reports only on interrupt endpoint `0x83`, which
  macOS `libusb` does not deliver (the vendor driver used IOKit directly). Capture is
  therefore triggered from the UI / Spacebar instead.

## Disclaimer

Independent, non-commercial, hobbyist interoperability project. **Not affiliated with,
authorized, or endorsed by AnMo Electronics / Dino-Lite or Sonix.** "Dino-Lite" and other
trademarks belong to their respective owners and are used here only nominatively, to
describe hardware compatibility. No vendor firmware, drivers, binaries, logos, or
proprietary source are included or redistributed; the app icon is original artwork. The
USB protocol is implemented from the GPL-2.0 Linux `gspca sn9c20x` driver. Provided "as
is", without warranty.

## License

GPL-2.0-or-later. The protocol is derived from the GPL-2.0 Linux `gspca sn9c20x`
driver — see [`LICENSE`](LICENSE). Upstream reference:
<https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/media/usb/gspca/sn9c20x.c>
