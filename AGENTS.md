# Repository Guidelines

## Project Structure & Module Organization

This is a compact macOS native project for the Dino-Lite AM411T microscope. Core files live at the repository root:

- `dino_metal.m`: Cocoa/Metal GUI viewer, libusb streaming, color pipeline, and PNG capture.
- `dino_grab.c`: headless RAW Bayer grabber and simple debayer output.
- `dino_shot.c`: single-frame PNG capture path.
- `gspca.h` and `sn9c20x.c`: protocol reference material derived from the Linux driver.
- `README.md`: setup, controls, quirks, and licensing notes.

Build outputs (`dino_metal`, `dino_grab`, `dino_shot`) are generated in the root and should not be treated as source assets.

## Build, Test, and Development Commands

- `make`: builds `dino_metal`, `dino_grab`, and `dino_shot`.
- `make run`: builds and launches the GUI viewer.
- `make clean`: removes generated binaries.
- `./dino_grab`: captures sample PPM/PGM frames to `/tmp`.
- `./dino_grab - | ffplay -f rawvideo -pixel_format rgb24 -video_size 1280x1023 -i -`: streams CLI output to `ffplay`.

Install libusb first with `brew install libusb`. The Makefile defaults to `BREW=/opt/homebrew`; override it when needed, for example `make BREW=/usr/local`.

## Coding Style & Naming Conventions

Use the existing C/Objective-C style: compact helpers, `snake_case` symbols, uppercase constants such as `VID`, `PID`, `FRAME_BYTES`, and `g_` prefixes for global runtime state. Keep indentation consistent with nearby code, generally four spaces in multi-line blocks. Prefer small static helpers for USB transport, frame assembly, color analysis, and rendering. Add comments only for protocol details, hardware quirks, or non-obvious image-processing choices.

## Testing Guidelines

There is no automated test suite. At minimum, run `make clean` then `make` before submitting changes. For behavior changes, smoke-test with the microscope attached: launch `./dino_metal`, confirm live preview, LED toggle, Bayer phase, contrast, and PNG capture; then run `./dino_grab` and verify frames in `/tmp`. Note hardware or macOS limits in the PR.

## Commit & Pull Request Guidelines

Recent history uses short, descriptive commit subjects, including conventional-style prefixes such as `docs:` when useful. Keep subjects imperative or clearly descriptive, for example `docs: add trademark disclaimer` or `Fix LED shutdown path`.

Pull requests should include the reason for the change, affected binaries or workflows, build/smoke-test results, and screenshots or captured output when UI or image quality changes. Link related issues when available and avoid committing generated binaries.

## Security & Configuration Tips

Do not vendor proprietary firmware, vendor drivers, or Dino-Lite assets. Keep USB protocol changes traceable to GPL-compatible sources and preserve the project disclaimer and GPL-2.0-or-later licensing context.
