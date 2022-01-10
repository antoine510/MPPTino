# Board setup

## Burn fuses

`bin/avrdude.exe -C etc/avrdude.conf -p m328p -c usbasp -U hfuse:w:0xda:m -U efuse:w:0xfd:m -U lfuse:w:0x62:m -B 5`

## Burn bootloader

`bin/avrdude.exe -C etc/avrdude.conf -p m328p -c usbasp -U flash:w:"../../../../../Users/antoi/OneDrive/Projects/MPPT/MPPTino/build/MPPTino.ino.with_bootloader.hex" -B 5`
