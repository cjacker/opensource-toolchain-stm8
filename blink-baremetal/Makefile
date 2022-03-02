all:
	sdcc -lstm8 -mstm8 blink.c
debug: 
	sdcc -lstm8 -mstm8 blink.c --out-fmt-elf --all-callee-saves --debug --verbose --stack-auto --fverbose-asm  --float-reent --no-peep

serial: all
	sudo stm8gal -p /dev/ttyUSB0 -w blink.ihx -reset 0 

swim: all
	sudo stm8flash -cstlinkv2 -pstm8s208mb -w blink.ihx

clean:
	rm -rf *.rst *.rel *.ihx *.sym *.asm *.map *.lk *.lst *.elf *.cdb
