#include <ST7565.h>
#include <MCP4716.h>

enum Pins {
	PIN_VIN = A1,
	PIN_VOUT = A2,
	PIN_IOUT = A3
};

static constexpr const int serial_identity = 0x01;
static constexpr const byte cmd_magic[] = {0x4f, 0xc7, 0xb2, 0x9a};
enum SerialState : uint8_t {
	MAGIC1, MAGIC2, MAGIC3, MAGIC4, IDENTITY, COMMAND
} serial_state;
enum CommandID : uint8_t {
	NONE,
	READ_ALL,
	SET_MPP_VOLTAGE_DV,	// Manually set MPP voltage in decivolts
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
void SetMPPVoltage(uint16_t mppv_dv) {
	dac.SetValue((mppv_dv * 17) / 10 + 196);
}

void setup() {
	Serial.begin(9600);
	serial_state = MAGIC1;

	pinMode(PIN_VIN, INPUT);
	pinMode(PIN_VOUT, INPUT);
	pinMode(PIN_IOUT, INPUT);

	ST7565::begin(0x03);
	ST7565::clear();

	ST7565::drawchar_aligned(2, 0, '.');
	ST7565::drawchar_aligned(4, 0, 'V');

	ST7565::drawchar_aligned(2, 2, '.');
	ST7565::drawchar_aligned(4, 2, 'V');

	ST7565::drawchar_aligned(2, 4, '.');
	ST7565::drawchar_aligned(5, 4, 'A');

	ST7565::drawchar_aligned(4, 6, 'W');

	SetMPPVoltage(320);
}

constexpr unsigned long stateUpdatePeriod = 1000ul;

void runCommand(CommandID command) {
	uint16_t mppv_dv;
	switch(command) {
	case READ_ALL:
		sdata.eout_j = eout_mj / 1000;
		eout_mj = 0;
		Serial.write((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case SET_MPP_VOLTAGE_DV:
		Serial.readBytes((uint8_t*)&mppv_dv, sizeof(mppv_dv));
		SetMPPVoltage(mppv_dv);
		break;
	}
	serial_state = MAGIC1;	// Done with command, reset serial state
}

unsigned long lastStateUpdate = 0;
void updateState() {
	sdata.vin_cv = (uint16_t)analogRead(PIN_VIN) * 45 / 10;
	sdata.vout_dv = (uint32_t)analogRead(PIN_VOUT) * 10264 / 10000;
	sdata.iout_ca = (uint16_t)(analogRead(PIN_IOUT) + 2) * 14 / 9;
	sdata.pout_dw = (uint32_t)sdata.vout_dv * sdata.iout_ca / 100;

	eout_mj += (uint32_t)sdata.pout_dw * (millis() - lastStateUpdate) / 10;
	lastStateUpdate = millis();

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

void serialEvent() {
	do {
		switch(serial_state) {
		case MAGIC1:
			if(Serial.read() == cmd_magic[0]) serial_state = MAGIC2;
			break;
		case MAGIC2:
			if(Serial.read() == cmd_magic[1]) serial_state = MAGIC3;
			else serial_state = MAGIC1;
			break;
		case MAGIC3:
			if(Serial.read() == cmd_magic[2]) serial_state = MAGIC4;
			else serial_state = MAGIC1;
			break;
		case MAGIC4:
			if(Serial.read() == cmd_magic[3]) serial_state = IDENTITY;
			else serial_state = MAGIC1;
			break;
		case IDENTITY:
			if(Serial.read() == serial_identity) serial_state = COMMAND;
			else serial_state = MAGIC1;
			break;
		case COMMAND:
			runCommand((CommandID)Serial.read());
			break;
		}
	} while(Serial.available());
}

void loop() {
	if(millis() - lastStateUpdate > stateUpdatePeriod) updateState();
}
