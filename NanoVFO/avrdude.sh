sudo avrdude -c stk200 -p m328p -U flash:w:NanoVFO.ino.with_bootloader.standard.hex:i -U hfuse:w:0xDE:m -U lfuse:w:0xFF:m -U efuse:w:0x05:m

