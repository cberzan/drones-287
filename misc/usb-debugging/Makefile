all: resetusb usbreset hub-ctrl

resetusb: resetusb.c
	gcc -o resetusb resetusb.c -lusb

usbreset: usbreset.c
	gcc -o usbreset usbreset.c

hub-ctrl: hub-ctrl.c
	gcc -o hub-ctrl hub-ctrl.c -lusb
