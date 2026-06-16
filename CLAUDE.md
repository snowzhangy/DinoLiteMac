# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

macOS (Apple Silicon) viewer for the **Dino-Lite AM411T** USB microscope (USB `a168:0615` =
Sonix **SN9C201** bridge + Micron/Aptina **MT9M111** sensor). The interface is vendor-specific
(class `0xFF`), so macOS won't bind it as a UVC camera; we drive it directly over `libusb`. The
USB control protocol is ported from the Linux kernel `gspca sn9c20x` driver.

## Build & run

```sh
brew install libusb        # one-time dependency
make                       # builds dino_metal (GUI) + dino_grab (CLI)
make run                   # or ./dino_metal
./dino_grab                # headless: 3 PPM frames (+ raw PGM) to /tmp (simple debayer)
./dino_grab - | ffplay -f rawvideo -pixel_format rgb24 -video_size 1280x1023 -i -
./dino_shot [out.png] [sat] [contrast] [gamma] [phase]   # single true-color PNG, tunable
make clean
```

No test suite. Verification is manual: plug in the scope, run, look at the image. If the app
can't claim the device, unplug/replug it (a SIGKILLed prior process can leave it mid-claim;
`dino_grab`/`dino_shot` call `libusb_reset_device` to recover, the GUI does not).

**Tuning the color pipeline:** use `dino_shot` — it runs the *same* AE/AWB/debayer as the GUI but
takes the color params as CLI args, so you can sweep saturation/contrast/gamma and read the PNG
back without recompiling (e.g. `./dino_shot /tmp/a.png 0.70 1.0 2.15`). It prints the converged
exposure, WB gains and auto-levels to stderr. Only one process can hold the device at a time, so
kill the GUI before running `dino_shot`.

`BREW ?= /opt/homebrew` in the Makefile is the libusb include/lib root — override for a different
Homebrew prefix.

## Architecture

Three binaries share the **same device protocol**:

- **`dino_metal.m`** — the real app. Objective-C + Cocoa + Metal, built with ARC (`-fobjc-arc`).
- **`dino_shot.c`** — single-frame grabber that mirrors `dino_metal`'s **full color pipeline**
  (AE + AWB + true-color debayer), with the color params exposed as CLI args. The tool for
  tuning/regression-checking color.
- **`dino_grab.c`** — minimal C grabber with a *simple* nearest-neighbour debayer, no AE/AWB.
  Useful for isolating whether a problem is in the USB/protocol layer vs. the color/render layer.

`sn9c20x.c` and `gspca.h` are the **GPL-2.0 Linux kernel reference sources** — read-only, gitignored,
not part of the build. Consult them when changing register/init tables.

### Data flow (dino_metal.m)

```
USB engine thread (engine())  ──shared_raw[] + mutex──>  Metal draw (drawInMTKView:)  ──> screen
   isochronous EP 0x81           ├─ AWB tick (CPU) 0.4s   GPU debayer/WB/levels/gamma shader
   feed()/emit() reassemble       └─ AE in engine loop
```

- A dedicated **pthread** (`engine`) owns all libusb I/O: init, isochronous streaming, LED
  changes, **software auto-exposure**, teardown. The main thread never touches the device.
- Frames arrive as iso packets; `feed()` detects the 6-byte frame header `FHDR` and `emit()`
  copies a complete `FRAME_BYTES` raw Bayer frame into `shared_raw` under `lock`.
- The Metal `Renderer` (an `MTKView` delegate) pulls the latest `shared_raw` each draw, uploads
  it to an `R8Unorm` texture, and the fragment shader (`kShader`, embedded source string) runs
  the color pipeline on the GPU.
- Globals prefixed `g_` (`g_led`, `g_gray`, `g_phase`, `g_gR/gG/gB`, `g_black/white/gamma/sat`,
  `g_contrast`, `g_exp`, `g_capture_req`) are the UI↔engine↔shader bus. Most are `volatile`; the
  raw frame is the only thing behind the mutex.

#### The color pipeline (the thing that goes wrong) — implemented THREE times, keep in sync

The proven "True Color" path (ported from `librepods/tools/dinolite_live.m` / `dinolite_snap.c`)
lives in **(1)** the GPU `kShader`, **(2)** `cpu_debayer` in `dino_metal.m` (used only for PNG
capture), and **(3)** `dino_shot.c`. All three must produce the same result — change one, change
all three. The stages:

1. **Software AE** (`compute_ae`, engine thread) drives MT9M111 shutter reg `0x09` to hold scene
   luma in a target band (parses a coarse luma from the Sonix frame header, falls back to a
   subsample). The sensor's own AE alone over/under-exposes this optic — this loop is required.
2. **Green-referenced white-patch AWB** (`compute_awb`): gains `gR=mG/mR`, `gB=mG/mB`, `gG=1`,
   sampled on near-neutral cells, plus auto **black/white levels** from the luma histogram.
   Converge-then-hold (counter `g_awb_n`); "Lock WB" / phase change re-triggers it.
3. **Render** (`cell_rgb` → chroma re-keyed onto full-res `luma_detail` → saturation with
   highlight roll-off → `tone` = black/white levels + gamma + final contrast).

## Hard-won hardware facts (do not "fix" these)

- **SXGA 1280×1023 is RAW Bayer only.** The bridge's hardware YUV/JPEG demosaic is capped at
  ≤640×480, so full resolution *must* be debayered in software/GPU. `W=1280 H=1024` but the
  bridge emits only **`CH=1023`** rows — window registers use 1024, framebuffers use 1023.
- **Set a neutral hardware baseline at stream start** (`dino_start`): MT9M111 color gains
  `0x2c–0x2f = 0x40` (R/G1/G2/B) + exposure `0x09 = g_exp`. Without this the gains float and the
  image takes a heavy color cast. Software AE then drives `0x09` from there (one-shot baseline,
  then a slow loop — that does NOT pump; what pumps is fighting the *sensor's* AE, which is why
  the dependence is on `0x09` shutter only). **History:** an earlier build dropped these writes
  and relied solely on the sensor's hardware AE/AGC → washed-out, cast color. That was the bug.
- **Keep hardware AWB ON** (MT9M111 op-mode `0x708e` in `mt9m111_init`). With it off the RAW red
  channel is starved ~5× and software can't recover it. The software AWB (`compute_awb`) refines
  on top of it (green-referenced, converge-then-hold).
- **The physical touch button cannot be read.** It only reports on interrupt EP `0x83`, which
  macOS libusb never delivers. Capture is triggered from the UI / **Spacebar** instead.
- **Bayer phase is runtime-selectable** (default `3`=BGGR) because a different unit may ship a
  different phase. `g_phase` indexes the same 0=GBRG 1=GRBG 2=RGGB 3=BGGR mapping in the AWB,
  CPU debayer, and shader — keep all three consistent.

## Protocol primitives (shared by both binaries)

`reg_w`/`reg_r` are vendor control transfers (request `0x08`/`0x00`); `i2c_w`/`i2c_w2` talk to the
MT9M111 over the bridge's I2C bridge at `0x10c0`. The `bridge_init[]` and `mt9m111_init[]` tables
plus `dino_start()` are lifted from `sn9c20x.c` — when porting changes, cross-check against that
reference rather than guessing register meanings.

## License constraint

GPL-2.0-or-later (protocol derived from the GPL kernel driver). Do not add vendor firmware,
binaries, logos, or proprietary source.
