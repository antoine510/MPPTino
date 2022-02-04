#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <Wire.h>
#include <PAC1710.h>
#include <ST7565.h>
#include <LowPower.h>

const uint8_t MCP4561address = 0x2e;
uint8_t MCPWiper = 0;
//const uint8_t trackingSpeed = 4u;  // Maximum change of wiper: 4 steps/s, one step is approx. 0.1A
PAC1710 pac;
typedef PAC1710::ValueReader<2, PAC1710::SS_20MV> PACReader;
constexpr uint8_t SERIAL_QUERY_PIN = 3;
uint8_t sleep = true;

struct SerialData {
	uint16_t millivolts;
	uint16_t milliamps;
	uint16_t deciwatts;
	uint16_t joules;
};
SerialData sdata;
uint8_t sendNow = false;

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
	MCPWiper += 1 - 2 * decrement;
}

void serial_query_ISR() {
	if(sleep) return;
	sendNow = true;
}

void setup() {
	ADCSRA &= ~(1 << ADEN); // Disable ADC

	Serial.begin(9600);
	Wire.begin();
	setWiper(MCPWiper);
	pac.SetSamplingTimesMs(20, 80);
	pac.SetAveraging(PAC1710::AVG_NONE, PAC1710::AVG_8);
	pac.SetSenseScale(PAC1710::SS_20MV);

	ST7565::begin(0x03);
	ST7565::clear();

	ST7565::drawchar_aligned(2, 0, '.');
	ST7565::drawchar_aligned(4, 0, 'V');

	ST7565::drawchar_aligned(2, 1, '.');
	ST7565::drawchar_aligned(5, 1, 'A');

	ST7565::drawchar_aligned(3, 2, 'W');

	ST7565::drawchar_aligned(3, 3, 'U');

	attachInterrupt(digitalPinToInterrupt(SERIAL_QUERY_PIN), &serial_query_ISR, RISING);
}

constexpr unsigned crashVoltage = 25000, minVoltage = 30900, maxVoltage = 32000;
constexpr unsigned long loopTime = 500ul;
unsigned loopActualTime;
byte crashCount = 0;
constexpr byte sleepCrashCount = 5; // If we have been under the crash voltage for more than sleepCrashCount * loopTime, sleep

void loop() {
	static unsigned long lastLoop = millis();
	loopActualTime = millis() - lastLoop;
	lastLoop = millis();

	pac.ReadOnce(PAC1710::READ_ALL);
	sdata.millivolts = PACReader::GetVoltageI(pac);
	sdata.milliamps = abs(PACReader::GetCurrentI(pac));
	sdata.deciwatts = PACReader::GetPowerI(pac);

	sdata.joules += (uint32_t)sdata.deciwatts * loopActualTime / 10000;

	if(sdata.millivolts < crashVoltage) {
		if(crashCount > sleepCrashCount) {
			if(!sleep) {
				pac.SetStandby(true);	// Sleep if running
				sleep = true;
			}
		} else {
			if(!crashCount) {
				setWiper(0);
			} else {
				++crashCount;
			}
		}
	} else {
		if(sdata.millivolts < minVoltage) moveWiper(true);
		else if(sdata.millivolts > maxVoltage) moveWiper(false);

		crashCount = 0;
		if(sleep) {
			pac.SetStandby(false);	// Wakeup if sleeping
			sleep = false;
		}
	}

	if(sendNow) {
		Serial.write((unsigned char*)(&sdata), sizeof(sdata));
		sdata.joules = 0;
		sendNow = false;
	}

	ST7565::drawchar_aligned(0, 0, sdata.millivolts / 10000 + 48);
	ST7565::drawchar_aligned(1, 0, (sdata.millivolts / 1000) % 10 + 48);
	ST7565::drawchar_aligned(3, 0, (sdata.millivolts / 100) % 10 + 48);

	ST7565::drawchar_aligned(0, 1, sdata.milliamps / 10000 + 48);
	ST7565::drawchar_aligned(1, 1, (sdata.milliamps / 1000) % 10 + 48);
	ST7565::drawchar_aligned(3, 1, (sdata.milliamps / 100) % 10 + 48);
	ST7565::drawchar_aligned(4, 1, (sdata.milliamps / 10) % 10 + 48);

	ST7565::drawchar_aligned(0, 2, (sdata.deciwatts / 1000) % 10 + 48);
	ST7565::drawchar_aligned(1, 2, (sdata.deciwatts / 100) % 10 + 48);
	ST7565::drawchar_aligned(2, 2, (sdata.deciwatts / 10) % 10 + 48);

	ST7565::drawchar_aligned(0, 3, MCPWiper / 100 + 48);
	ST7565::drawchar_aligned(1, 3, (MCPWiper / 10) % 10 + 48);
	ST7565::drawchar_aligned(2, 3, MCPWiper % 10 + 48);

	ST7565::display();

	if(sleep) LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_ON);
	else delay(loopTime);
}
