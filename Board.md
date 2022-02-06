# Board setup

## Burn fuses

`bin/avrdude.exe -C etc/avrdude.conf -p m328p -c usbasp -U hfuse:w:0xdf:m -U efuse:w:0xfd:m -U lfuse:w:0x62:m -B 5`
