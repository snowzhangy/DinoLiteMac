#!/bin/sh
# Build a self-contained, distributable DinoLiteMac.app and zip it for a release.
#
# Bundles libusb (LGPL-2.1) into the app so end users do NOT need Homebrew. The
# app is ad-hoc signed (arm64 requires a valid signature to run); it is NOT
# notarized, so a downloaded copy is quarantined - users clear it on first launch
# (right-click > Open, or `xattr -dr com.apple.quarantine DinoLiteMac.app`).
#
# Usage: ./pack-app.sh [version]   (BREW overrides the Homebrew prefix)
set -eu

BREW="${BREW:-/opt/homebrew}"
VERSION="${1:-1.1.0}"
APP="DinoLiteMac.app"
C="$APP/Contents"
LIBUSB="$(readlink -f "$BREW/lib/libusb-1.0.0.dylib")"
OLD_ID="$(otool -D "$LIBUSB" | tail -1)"

make dino_metal

rm -rf "$APP"
mkdir -p "$C/MacOS" "$C/Frameworks" "$C/Resources"
cp dino_metal "$C/MacOS/dino_metal"
cp "$LIBUSB" "$C/Frameworks/libusb-1.0.0.dylib"
chmod u+w "$C/Frameworks/libusb-1.0.0.dylib"

# Point the executable at the bundled libusb (@rpath -> Contents/Frameworks).
install_name_tool -id   @rpath/libusb-1.0.0.dylib            "$C/Frameworks/libusb-1.0.0.dylib"
install_name_tool -change "$OLD_ID" @rpath/libusb-1.0.0.dylib "$C/MacOS/dino_metal"
install_name_tool -add_rpath @loader_path/../Frameworks       "$C/MacOS/dino_metal"
# Drop the build-time Homebrew rpath so the app uses ONLY its bundled libusb (deterministic, no brew).
install_name_tool -delete_rpath "$BREW/lib" "$C/MacOS/dino_metal" 2>/dev/null || true

# libusb license text - required when redistributing the dylib (LGPL-2.1).
cp "$(dirname "$LIBUSB")/../COPYING" "$C/Resources/libusb-COPYING.txt"

cat > "$C/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>DinoLiteMac</string>
  <key>CFBundleDisplayName</key><string>DinoLiteMac</string>
  <key>CFBundleIdentifier</key><string>org.dinolitemac.viewer</string>
  <key>CFBundleVersion</key><string>$VERSION</string>
  <key>CFBundleShortVersionString</key><string>$VERSION</string>
  <key>CFBundleExecutable</key><string>dino_metal</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>LSMinimumSystemVersion</key><string>12.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>LSApplicationCategoryType</key><string>public.app-category.photography</string>
</dict>
</plist>
PLIST

# arm64 needs a valid signature to execute. Sign the nested dylib first, then the app.
codesign -f -s - "$C/Frameworks/libusb-1.0.0.dylib"
codesign -f -s - "$APP"
codesign --verify --deep --strict "$APP"

ZIP="DinoLiteMac-v$VERSION-macOS-arm64.zip"
rm -f "$ZIP"
ditto -c -k --sequesterRsrc --keepParent "$APP" "$ZIP"
echo "built $APP and $ZIP"
