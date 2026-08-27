PORT:=$(shell ls /dev/tty.usb* ) 

.PHONY: init build screen 




init:
	cmake --preset Debug

build:
	cmake --build --preset Debug

release:
	cmake --preset Release
	cmake --build  --preset Release

screen:
	screen $(PORT) 115200

clean: 
	rm -rf build | true