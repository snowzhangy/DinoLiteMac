# Makefile for DinoLiteMac
# Requires libusb (brew install libusb).

BREW       ?= /opt/homebrew
CFLAGS     ?= -O2 -Wall
INC        := -I$(BREW)/include
LIB        := -L$(BREW)/lib -lusb-1.0 -Wl,-rpath,$(BREW)/lib
FRAMEWORKS := -framework Cocoa -framework Metal -framework MetalKit \
              -framework CoreGraphics -framework ImageIO

all: dino_metal dino_grab dino_shot

# Metal GUI viewer
dino_metal: dino_metal.m
	clang $(CFLAGS) $< -o $@ $(INC) $(LIB) $(FRAMEWORKS) -fobjc-arc

# headless CLI grabber (raw debayer)
dino_grab: dino_grab.c
	cc $(CFLAGS) $< -o $@ $(INC) $(LIB)

# single-frame grabber sharing dino_metal's true-color pipeline (PNG out, tunable via argv)
dino_shot: dino_shot.c
	cc $(CFLAGS) $< -o $@ $(INC) $(LIB) -framework CoreGraphics -framework ImageIO -framework CoreFoundation

run: dino_metal
	./dino_metal

# self-contained, ad-hoc-signed .app + release zip (bundles libusb; see pack-app.sh)
VERSION ?= 1.1.0
dist:
	BREW=$(BREW) ./pack-app.sh $(VERSION)

clean:
	rm -f dino_metal dino_grab dino_shot
	rm -rf DinoLiteMac.app DinoLiteMac-v*-macOS-arm64.zip

.PHONY: all run dist clean
