#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <Wire.h>
#include <PAC1710.h>
#include <ST7565.h>
#include <LowPower.h>

static constexpr const int serial_identity = 0x01;
static constexpr const byte cmd_magic[] = {0x4f, 0xc7, 0xb2, 0x9a};
enum SerialState : uint8_t {
	MAGIC1, MAGIC2, MAGIC3, MAGIC4, IDENTITY, COMMAND
} serial_state;
enum CommandID : uint8_t {
	READ_ALL = 0x01,
	SET_MAX_WIPER = 0x02,		// Max wiper is next byte
	SET_OUTPUT_ENABLED = 0x04	// 0b0000010e, e is output enable bit
};

const uint8_t MCP4561address = 0x2e;
uint8_t max_wiper = 0xff;
uint8_t MCPWiper = 0;

PAC1710 pac;
typedef PAC1710::ValueReader<2, PAC1710::SS_80MV> PACReader;
bool sleep = false;

struct SerialData {
	uint16_t millivolts;
	uint16_t milliamps;
	uint16_t deciwatts;
	uint16_t joules;
};
SerialData* sdata, *sdataLast;
uint16_t global_joules = 0;

void setWiper(uint8_t value) {
	Wire.beginTransmission(MCP4561address);
	Wire.write(0x00); // Write command to address 0 (wiper volatile)
	Wire.write(value);
	Wire.endTransmission();
	MCPWiper = value;
}

void moveWiper(bool decrement) {
	Wire.beginTransmission(MCP4561address);
	Wire.write(0x04 << decrement); // Inc/Dec address 0 (wiper volatile)
	Wire.endTransmission();
	if(!(decrement && MCPWiper == 0) && !(!decrement && MCPWiper == max_wiper))
		MCPWiper += 1 - 2 * decrement;
}


void setup() {
	ADCSRA &= ~(1 << ADEN); // Disable ADC

	Serial.begin(9600);
	Serial.setTimeout(200);
	serial_state = MAGIC1;
	Wire.begin();
	setWiper(0);
	pac.SetSamplingTimesMs(20, 80);
	pac.SetAveraging(PAC1710::AVG_NONE, PAC1710::AVG_8);
	pac.SetSenseScale(PAC1710::SS_80MV);

	sdata = new SerialData{};
	sdataLast = new SerialData{};

	ST7565::begin(0x03);
	ST7565::clear();

	ST7565::drawchar_aligned(2, 0, '.');
	ST7565::drawchar_aligned(4, 0, 'V');

	ST7565::drawchar_aligned(2, 2, '.');
	ST7565::drawchar_aligned(5, 2, 'A');

	ST7565::drawchar_aligned(4, 4, 'W');
}

constexpr unsigned crashVoltage = 25000;
constexpr unsigned minVoltage = 30000, maxVoltage = 35000;
bool wiper_locked = false;

/**
 * @brief Modulates reactivity to over or under voltage with power
 * 2^n only, higher is less reactive at high power
 * Ex: V > maxV => MCP += 1 + MCP / power_reactivity_scaling
 */
constexpr uint8_t power_reactivity_scaling = 32;

// True for descreasing power, false for increasing power
bool exploration_direction = false;
uint8_t before_crash_wiper = 0;

constexpr unsigned long stateUpdatePeriod = 1000ul;
byte crashCount = 0;
constexpr byte sleepCrashCount = 5; // If we have been under the crash voltage for more than sleepCrashCount * loopTime, sleep

void decreasePower() {
	uint8_t dec = 1 + MCPWiper / power_reactivity_scaling;	// Decrease amperage proportionally to current power
	if(MCPWiper <= dec) setWiper(0);
	else setWiper(MCPWiper - dec);
}
void increasePower() {
	uint8_t inc = 1 + MCPWiper / power_reactivity_scaling;	// Increase amperage proportionally to current power
	if((max_wiper - MCPWiper) <= inc) setWiper(max_wiper);
	else setWiper(MCPWiper + inc);
}

void runCommand(CommandID command) {
	switch(command) {
	case READ_ALL:
		sdata->joules = global_joules;
		Serial.write((byte*)(sdata), sizeof(SerialData));
		global_joules = 0;
		break;
	case SET_MAX_WIPER:
		Serial.readBytes(&max_wiper, 1);
		break;
	case SET_OUTPUT_ENABLED:
	case SET_OUTPUT_ENABLED+1:
		wiper_locked = !(command & 0b00000001);
		if(wiper_locked) setWiper(0);
		break;
	}
	serial_state = MAGIC1;	// Done with command, reset serial state
}

unsigned long lastStateUpdate = 0;
void updateState() {
	pac.ReadOnce(PAC1710::READ_ALL);
	sdata->millivolts = PACReader::GetVoltageI(pac);
	sdata->milliamps = abs(PACReader::GetCurrentI(pac));
	sdata->deciwatts = PACReader::GetPowerI(pac);

	global_joules += (uint32_t)sdata->deciwatts * (millis() - lastStateUpdate) / 10000;
	lastStateUpdate = millis();

	if(!wiper_locked) {
		if(sdataLast->millivolts > crashVoltage && sdata->millivolts < crashVoltage) {
			before_crash_wiper = MCPWiper;
			setWiper((uint32_t)sdata->millivolts * MCPWiper / sdataLast->millivolts);
		} else if(sdata->millivolts < minVoltage) {	// Under-production
			decreasePower();
			exploration_direction = true;
		} else if(before_crash_wiper > 0) {
			setWiper(before_crash_wiper - 1 - before_crash_wiper / power_reactivity_scaling);
			before_crash_wiper = 0;
		} else if(sdata->millivolts > maxVoltage) {	// Over-production/Ouput limited
			increasePower();
			exploration_direction = false;
		} else {
			// Random walk within [minV, maxV]
			if(sdataLast->deciwatts > sdata->deciwatts) {
				exploration_direction = !exploration_direction;
			}
			moveWiper(exploration_direction);
		}
	}

	if(!MCPWiper) {	// Handle going to sleep
		if(crashCount > sleepCrashCount) {
			if(!sleep) {
				pac.SetStandby(true);	// Sleep if running
				serial_state = MAGIC1;	// Reset serial state
				sleep = true;
			}
		} else {
			++crashCount;
		}
	} else {
		if(sleep) {
			pac.SetStandby(false);	// Wakeup if sleeping
			sleep = false;
		}
		crashCount = 0;
	}

	byte tensOfVolts = sdata->millivolts / 10000;
	ST7565::drawchar_aligned(0, 0, tensOfVolts ? tensOfVolts + 48 : 0);
	ST7565::drawchar_aligned(1, 0, (sdata->millivolts / 1000) % 10 + 48);
	ST7565::drawchar_aligned(3, 0, (sdata->millivolts / 100) % 10 + 48);

	byte tensOfAmps = sdata->milliamps / 10000;
	ST7565::drawchar_aligned(0, 2, tensOfAmps ? tensOfAmps + 48 : 0);
	ST7565::drawchar_aligned(1, 2, (sdata->milliamps / 1000) % 10 + 48);
	ST7565::drawchar_aligned(3, 2, (sdata->milliamps / 100) % 10 + 48);
	ST7565::drawchar_aligned(4, 2, (sdata->milliamps / 10) % 10 + 48);

	byte thousandsOfWatts = sdata->deciwatts / 10000;
	byte hundredsOfWatts = (sdata->deciwatts / 1000) % 10;
	byte tensOfWatts = (sdata->deciwatts / 100) % 10;
	ST7565::drawchar_aligned(0, 4, thousandsOfWatts ? thousandsOfWatts + 48 : 0);
	ST7565::drawchar_aligned(1, 4, (hundredsOfWatts || thousandsOfWatts) ? hundredsOfWatts + 48 : 0);
	ST7565::drawchar_aligned(2, 4, (tensOfWatts || hundredsOfWatts || thousandsOfWatts) ? tensOfWatts + 48 : 0);
	ST7565::drawchar_aligned(3, 4, (sdata->deciwatts / 10) % 10 + 48);

	byte hundredsOfMCP = MCPWiper / 100;
	byte tensOfMCP = (MCPWiper / 10) % 10;
	ST7565::drawchar_aligned(0, 6, hundredsOfMCP ? hundredsOfMCP + 48 : 0);
	ST7565::drawchar_aligned(1, 6, (tensOfMCP || hundredsOfMCP) ? tensOfMCP + 48 : 0);
	ST7565::drawchar_aligned(2, 6, MCPWiper % 10 + 48);

	ST7565::display();

	SerialData* t = sdata;
	sdata = sdataLast;
	sdataLast = t;

	if(sleep) {
		// millis are not counted when sleeping.
		// We need to add 8000ms by decreasing lastStateUpdate by the same amount if possible.
		if(lastStateUpdate < 8000) lastStateUpdate = 0; else lastStateUpdate -= 8000;
		LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_ON);
	}
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
