PORT:=$(shell ls /dev/tty.usb* ) 
FORMAT_FILE := $(shell find Core/ -name "*.c" -or -name "*.h" -type f)

.PHONY: init build screen format


init:
	cmake --preset Debug

format: 
	clang-format -i ${FORMAT_FILE}

build:
	cmake --build --preset Debug

release:
	cmake --preset Release
	cmake --build  --preset Release

screen:
	screen $(PORT) 115200
	stty sane

clean: 
	rm -rf build | true