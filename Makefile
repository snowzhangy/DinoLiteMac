# Makefile for DinoLiteMac
# Requires libusb (brew install libusb).

BREW       ?= /opt/homebrew
CFLAGS     ?= -O2 -Wall
INC        := -I$(BREW)/include
LIB        := -L$(BREW)/lib -lusb-1.0 -Wl,-rpath,$(BREW)/lib
FRAMEWORKS := -framework Cocoa -framework Metal -framework MetalKit \
              -framework CoreGraphics -framework ImageIO

all: dino_metal dino_grab

# Metal GUI viewer
dino_metal: dino_metal.m
	clang $(CFLAGS) $< -o $@ $(INC) $(LIB) $(FRAMEWORKS) -fobjc-arc

# headless CLI grabber
dino_grab: dino_grab.c
	cc $(CFLAGS) $< -o $@ $(INC) $(LIB)

run: dino_metal
	./dino_metal

clean:
	rm -f dino_metal dino_grab

.PHONY: all run clean
