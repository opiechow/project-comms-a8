#when compiling you must name the compilation rule to compile
#when uploading you must name the upload rule to upload

target=uart_handler

compile:
	avr-gcc -std=c99 -g -Os -mmcu=atmega8 -c $(target).c -o $(target).o
	avr-gcc -g -mmcu=atmega8 -o $(target).elf $(target).o
	avr-objcopy -j .text -j .data -O ihex $(target).elf $(target).hex
	rm -f $(target).o $(target).elf

flash:
	avrdude -v -c usbasp -p atmega8 -u -U flash:w:$(target).hex:i

