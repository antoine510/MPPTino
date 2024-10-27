# EEPROM setup code

```
#include <EEPROM.h>

#define BOARD_ID 1

void setup() {
  EEPROM.write(0, BOARD_ID);
}
void loop() {}
```
