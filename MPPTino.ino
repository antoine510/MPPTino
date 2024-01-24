#include <EEPROM.h>
#include <ST7565.h>
#include <MCP4716.h>
#include <OneWire.h>	// For DS18B20

enum Pins {
	PIN_VIN = A1,
	PIN_VOUT = A2,
	PIN_IOUT = A3,
  PIN_TEMPERATURE = 9,
  PIN_TXEN = 2
};

enum CommandID : uint8_t {
	NONE,
	READ_ALL,
	SET_MPP_MANUAL_DV,	// Manually set MPP voltage in decivolts
  SET_MPP_AUTO,
  SET_MAX_TEMP_C  // Set maximum allowed temperature in celcius
};

struct SerialData {
	uint16_t vin_cv;
	uint16_t vout_dv;
	uint16_t iout_ca;
	uint16_t pout_dw;
	uint16_t eout_j;
	int8_t temp_c;
};
SerialData sdata;
uint32_t eout_mj = 0;

MCP4716 dac;
static constexpr const uint16_t dacMin = 672, dacMax = 876; // 28V - 40V
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) {
  return (mppv_dv * 17) / 10 + 196;
}
uint16_t dacValue = VoltageToDAC(320);  // 32V default MPP voltage
uint16_t manualMPP = 0; // 0 for automatic

void SetMPPVoltage(uint16_t mppv_dv) {
  manualMPP = mppv_dv;
  dacValue = VoltageToDAC(mppv_dv);
	dac.SetValue(dacValue);
}

bool NudgeMPP(bool increase) {
  dacValue += increase * 2 - 1;
  bool saturated = true;
  if(dacValue < dacMin) dacValue = dacMin;
  else if(dacValue > dacMax) dacValue = dacMax;
  else saturated = false;
  dac.SetValue(dacValue);
  return saturated;
}

int8_t maxTemperature = 80;

int8_t GetTemperature() {
	static OneWire tsens(PIN_TEMPERATURE);

	tsens.reset();
	tsens.skip();
	tsens.write(0xbe);
	byte l = tsens.read(), h = tsens.read();
	int8_t res = (int8_t)((l >> 4) | (h << 4));

	tsens.reset();
	tsens.skip();
	tsens.write(0x44);	// Start next measurement
	return res;
}

void setup() {
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
  Serial.write(data, len);
  Serial.flush();
  digitalWrite(PIN_TXEN, LOW);
}

constexpr unsigned long stateUpdatePeriod = 1000ul;

void runCommand(CommandID command) {
	uint16_t mppv_dv;
	switch(command) {
	case READ_ALL:
		sdata.eout_j = eout_mj / 1000;
		eout_mj = 0;
    SendRS485((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case SET_MPP_MANUAL_DV:
		Serial.readBytes((uint8_t*)&mppv_dv, sizeof(mppv_dv));
		SetMPPVoltage(mppv_dv);
		break;
  case SET_MPP_AUTO:
    manualMPP = 0;
    break;
  case SET_MAX_TEMP_C:
    maxTemperature = (int8_t)Serial.read();
    break;
	}
}

unsigned long lastStateUpdate = 0;
void updateState() {
  static bool nudge = true;
  static uint32_t lastPower = 0;
	sdata.vin_cv = (uint32_t)analogRead(PIN_VIN) * 199 / 44;
	sdata.vout_dv = (uint16_t)analogRead(PIN_VOUT) * 39 / 38;
	sdata.iout_ca = (uint16_t)(analogRead(PIN_IOUT) + 2) * 14 / 9;
  uint32_t pout_mw = (uint32_t)sdata.vout_dv * sdata.iout_ca;
	sdata.pout_dw = pout_mw / 100;
	sdata.temp_c = GetTemperature();

  if(sdata.temp_c > maxTemperature) {
    NudgeMPP(true);
  } else if(manualMPP && dacValue != VoltageToDAC(manualMPP)) {
    SetMPPVoltage(manualMPP);
  } else {
    if(pout_mw < lastPower) nudge = !nudge;
    bool saturated = NudgeMPP(nudge);
    if(saturated) nudge = !nudge;   // Switch direction on saturation
    lastPower = pout_mw;
  }

	eout_mj += (uint32_t)sdata.pout_dw * (millis() - lastStateUpdate) / 10;
	lastStateUpdate = millis();

	updateLCD();
}


enum SerialSeq : uint8_t {
	MAGIC1, MAGIC2, MAGIC3, MAGIC4, IDENTITY, COMMAND
} serial_state = MAGIC1;

uint8_t updateSerialState(uint8_t byte) {
  static constexpr const uint8_t cmd_magic[] = {0x4f, 0xc7, 0xb2, 0x9a};
  switch(serial_state) {
		case MAGIC1: return byte == cmd_magic[0] ? MAGIC2 : MAGIC1;
		case MAGIC2: return byte == cmd_magic[1] ? MAGIC3 : MAGIC1;
		case MAGIC3: return byte == cmd_magic[2] ? MAGIC4 : MAGIC1;
		case MAGIC4: return byte == cmd_magic[3] ? IDENTITY : MAGIC1;
		case IDENTITY: return byte == EEPROM.read(0) ? COMMAND : MAGIC1;
		case COMMAND: runCommand((CommandID)byte); return MAGIC1;
	}
}


void loop() {
	if(millis() - lastStateUpdate > stateUpdatePeriod) updateState();

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
	}
}

void setupLCD() {
  ST7565::begin(0x03);
	ST7565::clear();

	ST7565::drawchar_aligned(2, 0, '.');

	ST7565::drawchar_aligned(7, 0, '.');
	ST7565::drawchar_aligned(9, 0, 'V');

	ST7565::drawchar_aligned(3, 2, 'C');

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
	ST7565::drawchar_aligned(5, 0, tensOfVolts ? tensOfVolts + 48 : 0);
	ST7565::drawchar_aligned(6, 0, (sdata.vout_dv / 10) % 10 + 48);
	ST7565::drawchar_aligned(8, 0, sdata.vout_dv % 10 + 48);

	byte tempi = 0;
	int8_t temp = sdata.temp_c;
	if(temp < 0) {
		ST7565::drawchar_aligned(tempi++, 2, '-');
		temp = -temp;
	}
	byte tensOfDegrees = temp / 10;
	ST7565::drawchar_aligned(tempi++, 2, tensOfDegrees ? tensOfDegrees + 48 : 0);
	ST7565::drawchar_aligned(tempi++, 2, temp % 10 + 48);

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
