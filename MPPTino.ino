#include <EEPROM.h>
#include <MCP4716.h>

#define USE_LCD 1
#if USE_LCD
#include <ST7565.h>
#endif

static constexpr const uint8_t MAGIC_NUMBER = 0x42;
static constexpr const uint16_t mppVoltage_dV = 320;  // 32V default MPP voltage
static constexpr const unsigned long stateUpdatePeriod = 1000ul;

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
  SET_MPP_AUTO = 0x3
};

struct SerialData {
	uint16_t vin_cv;
	uint16_t vout_dv;
	uint16_t iout_ca;
	uint16_t pout_dw;
	uint16_t eout_j;
};
SerialData sdata;
uint32_t eout_dj = 0;

MCP4716 dac;
//static constexpr const uint16_t dacMin = 672, dacMax = 791; // 28V - 35V
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv * 17) / 10 + 196; }
uint16_t dacValue = VoltageToDAC(mppVoltage_dV);

constexpr uint16_t vinTransform_cV(uint16_t count) { return count * 50 / 11; }
constexpr uint16_t voutTransform_dV(uint16_t count) { return count * 39 / 38; }
constexpr uint16_t ioutTransform_cA(uint16_t count) { return count * 8 / 5; }

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

void setup() {
  PORTB |= 0x02;  // Enable LTC3813
  DDRB |= 0x02;

	Serial.begin(9600);

	pinMode(PIN_VIN, INPUT);
	pinMode(PIN_VOUT, INPUT);
	pinMode(PIN_IOUT, INPUT);

  pinMode(PIN_TXEN, OUTPUT);
  digitalWrite(PIN_TXEN, LOW);

#if USE_LCD
	setupLCD();
#endif

	dac.SetValue(dacValue); // Initialize DAC to startup value
}

void SendRS485(uint8_t* data, size_t len) {
  digitalWrite(PIN_TXEN, HIGH);
  delay(1);
  Serial.write(data, len);
  Serial.flush();
  digitalWrite(PIN_TXEN, LOW);
}

bool manualDisableOutput = false;
uint16_t manualMPP = 0; // 0 for automatic
void runCommand(CommandID command) {
	switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER));
    break;
	case READ_ALL:
		sdata.eout_j = eout_dj / 10;
		eout_dj = 0;
    SendRS485((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case SET_MPP_MANUAL_DV:
    Serial.readBytes((uint8_t*)&manualMPP, sizeof(manualMPP));
		SetMPPVoltage(manualMPP);
  	break;
  case SET_MPP_AUTO:
    manualMPP = 0;
    break;
	}
}

unsigned long nextStateUpdate = 0;
void updateState() {
  nextStateUpdate = millis() + stateUpdatePeriod;
//  static bool nudge = true;
//  static uint32_t lastPower = 0;
	sdata.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
	sdata.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
	sdata.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
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
    SetMPPVoltage(mppVoltage_dV);
  }

	eout_dj += (uint32_t)sdata.pout_dw;

#if USE_LCD
	updateLCD();
#endif
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


void loop() {
	if(millis() > nextStateUpdate) updateState();

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
	}
}

#if USE_LCD
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
#endif
