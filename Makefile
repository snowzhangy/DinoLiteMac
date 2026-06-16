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

clean:
	rm -f dino_metal dino_grab dino_shot

.PHONY: all run clean
