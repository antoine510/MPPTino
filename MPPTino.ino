#include <EEPROM.h>
#include <ST7565.h>
#include <MCP4716.h>

static constexpr const uint8_t MAGIC_NUMBER = 0x42;

enum Pins {
	PIN_VIN = A1,
	PIN_VOUT = A2,
	PIN_IOUT = A3,
  PIN_TXEN = 2,
  PIN_SHDN = 9  // PB1
};

enum CommandID : uint8_t {
	MAGIC = 0x0,
	READ_ALL = 0x1,
	SET_MPP_MANUAL_DV = 0x2,	// Manually set MPP voltage in decivolts
  SET_MPP_AUTO = 0x3,
  ENABLE_OUTPUT = 0x4,
  DISABLE_OUTPUT = 0x5,
};

struct SerialData {
	uint16_t vin_cv;
	uint16_t vout_dv;
	uint16_t iout_ca;
	uint16_t pout_dw;
	uint16_t eout_j;
};
SerialData sdata;
uint32_t eout_mj = 0;

MCP4716 dac;
//static constexpr const uint16_t dacMin = 672, dacMax = 791; // 28V - 35V
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) {
  return (mppv_dv * 17) / 10 + 196;
}
uint16_t dacValue = VoltageToDAC(320);  // 32V default MPP voltage
uint16_t manualMPP = 0; // 0 for automatic

bool manualDisableOutput = false;

void SetMPPVoltage(uint16_t mppv_dv) {
  auto newDACValue = VoltageToDAC(mppv_dv);
  if(newDACValue == dacValue) return;
  dacValue = newDACValue;
	dac.SetValue(dacValue);
}

/*bool NudgeMPP(bool increase) {
  dacValue += increase * 2 - 1;
  bool saturated = true;
  if(dacValue < dacMin) dacValue = dacMin;
  else if(dacValue > dacMax) dacValue = dacMax;
  else saturated = false;
  dac.SetValue(dacValue);
  return saturated;
}*/

void EnableOutput() {
  PORTB |= 0x02;
}
void DisableOutput() {
  PORTB &= 0xfd;
}

void setup() {
  PORTB &= 0xfd;  // Shutdown LTC3813
  DDRB |= 0x02;

	Serial.begin(9600);

	pinMode(PIN_VIN, INPUT);
	pinMode(PIN_VOUT, INPUT);
	pinMode(PIN_IOUT, INPUT);

  pinMode(PIN_TXEN, OUTPUT);
  digitalWrite(PIN_TXEN, LOW);

	setupLCD();

	dac.SetValue(dacValue); // Initialize DAC to startup value
}

void SendRS485(uint8_t* data, size_t len) {
  digitalWrite(PIN_TXEN, HIGH);
  delay(1);
  Serial.write(data, len);
  Serial.flush();
  digitalWrite(PIN_TXEN, LOW);
}

void runCommand(CommandID command) {
	switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER));
    break;
	case READ_ALL:
		sdata.eout_j = eout_mj / 1000;
		eout_mj = 0;
    SendRS485((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case SET_MPP_MANUAL_DV:
    Serial.readBytes((uint8_t*)&manualMPP, sizeof(manualMPP));
		SetMPPVoltage(manualMPP);
  	break;
  case SET_MPP_AUTO:
    manualMPP = 0;
    break;
  case ENABLE_OUTPUT:
    manualDisableOutput = false;
    break;
  case DISABLE_OUTPUT:
    manualDisableOutput = true;
    DisableOutput();
    break;
	}
}

bool isSleeping = true;
unsigned long lastOutputEnableCheck = 0, lastSleepSwitch = 0;
static constexpr const unsigned long wakeDelay_ms = 5000;
void checkEnableOutput() {
  if(manualDisableOutput) return;
  auto vin_cnt = analogRead(PIN_VIN), iout_cnt = analogRead(PIN_IOUT);
  unsigned long now = millis();
  if(vin_cnt > VoltageToDAC(450) || vin_cnt < VoltageToDAC(250)) { // Bad input voltage
    DisableOutput();
  } else {
    if(iout_cnt == 0) {
      if(isSleeping) {
        if(now - lastSleepSwitch > wakeDelay_ms) {
          isSleeping = false;
          lastSleepSwitch = now;
          EnableOutput();
        }
      } else {
        isSleeping = true;
        lastSleepSwitch = now;
        DisableOutput();
      }
    }
  }
  lastOutputEnableCheck = now;
}

unsigned long lastStateUpdate = 0;
void updateState() {
//  static bool nudge = true;
  static uint32_t lastPower = 0;
	sdata.vin_cv = (uint32_t)analogRead(PIN_VIN) * 199 / 44;
	sdata.vout_dv = (uint16_t)analogRead(PIN_VOUT) * 39 / 38;
	sdata.iout_ca = (uint16_t)analogRead(PIN_IOUT) * 8 / 5;
  if(sdata.iout_ca > 0) sdata.iout_ca += 5; // Compensates for input offset voltage of sense amplifier
  uint32_t pout_mw = (uint32_t)sdata.vout_dv * sdata.iout_ca;
	sdata.pout_dw = pout_mw / 100;

  if(manualMPP) {
    SetMPPVoltage(manualMPP);
//  } else if(pout_mw > 5000) {
//    if(pout_mw < lastPower) nudge = !nudge;
//    bool saturated = NudgeMPP(nudge);
//    if(saturated) nudge = !nudge;   // Switch direction on saturation
//    lastPower = pout_mw;
  } else {
    SetMPPVoltage(320);
  }

	eout_mj += (uint32_t)sdata.pout_dw * (millis() - lastStateUpdate) / 10;
	lastStateUpdate = millis();

	updateLCD();
}


enum SerialSeq : uint8_t {
	MAGIC1, MAGIC2, IDENTITY, COMMAND
} serial_state = MAGIC1;

uint8_t updateSerialState(uint8_t byte) {
  static constexpr const uint8_t cmd_magic[] = {0x4f, 0xc7};
  switch(serial_state) {
		case MAGIC1: return byte == cmd_magic[0] ? MAGIC2 : MAGIC1;
		case MAGIC2: return byte == cmd_magic[1] ? IDENTITY : MAGIC1;
		case IDENTITY: return byte == EEPROM.read(0) ? COMMAND : MAGIC1;
		case COMMAND: runCommand((CommandID)byte); return MAGIC1;
	}
}

static constexpr const unsigned long outputEnableCheckPeriod = 250ul;
static constexpr const unsigned long stateUpdatePeriod = 1000ul;
void loop() {
	if(millis() - lastOutputEnableCheck > outputEnableCheckPeriod) checkEnableOutput();
	if(millis() - lastStateUpdate > stateUpdatePeriod) updateState();

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
	}
}

void setupLCD() {
  ST7565::begin(0x03);
	ST7565::clear();

	ST7565::drawchar_aligned(2, 0, '.');
  ST7565::drawchar_aligned(4, 0, 'V');

	ST7565::drawchar_aligned(2, 2, '.');
	ST7565::drawchar_aligned(4, 2, 'V');

	ST7565::drawchar_aligned(2, 4, '.');
	ST7565::drawchar_aligned(5, 4, 'A');

	ST7565::drawchar_aligned(4, 6, 'W');
}

void updateLCD() {
  byte tensOfVolts = sdata.vin_cv / 1000;
	ST7565::drawchar_aligned(0, 0, tensOfVolts ? tensOfVolts + 48 : 0);
	ST7565::drawchar_aligned(1, 0, (sdata.vin_cv / 100) % 10 + 48);
	ST7565::drawchar_aligned(3, 0, (sdata.vin_cv / 10) % 10 + 48);

	tensOfVolts = sdata.vout_dv / 100;
	ST7565::drawchar_aligned(0, 2, tensOfVolts ? tensOfVolts + 48 : 0);
	ST7565::drawchar_aligned(1, 2, (sdata.vout_dv / 10) % 10 + 48);
	ST7565::drawchar_aligned(3, 2, sdata.vout_dv % 10 + 48);

	byte tensOfAmps = sdata.iout_ca / 1000;
	ST7565::drawchar_aligned(0, 4, tensOfAmps ? tensOfAmps + 48 : 0);
	ST7565::drawchar_aligned(1, 4, (sdata.iout_ca / 100) % 10 + 48);
	ST7565::drawchar_aligned(3, 4, (sdata.iout_ca / 10) % 10 + 48);
	ST7565::drawchar_aligned(4, 4, sdata.iout_ca % 10 + 48);

	byte thousandsOfWatts = sdata.pout_dw / 10000;
	byte hundredsOfWatts = (sdata.pout_dw / 1000) % 10;
	byte tensOfWatts = (sdata.pout_dw / 100) % 10;
	ST7565::drawchar_aligned(0, 6, thousandsOfWatts ? thousandsOfWatts + 48 : 0);
	ST7565::drawchar_aligned(1, 6, (hundredsOfWatts || thousandsOfWatts) ? hundredsOfWatts + 48 : 0);
	ST7565::drawchar_aligned(2, 6, (tensOfWatts || hundredsOfWatts || thousandsOfWatts) ? tensOfWatts + 48 : 0);
	ST7565::drawchar_aligned(3, 6, (sdata.pout_dw / 10) % 10 + 48);

	ST7565::display();
}
