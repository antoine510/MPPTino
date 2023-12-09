# Board setup

## Burn fuses

`bin/avrdude.exe -C etc/avrdude.conf -p m328pb -c usbasp -U hfuse:w:0xdf:m -U efuse:w:0xfd:m -U lfuse:w:0xE2:m -B 5`

## Write EEPROM

Use a simple EEPROM write example to write the MPPT ID at address 0 in the EEPROM.
