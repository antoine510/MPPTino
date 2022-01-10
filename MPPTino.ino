#include <Wire.h>
#include <PAC1710.h>
#include <ST7565.h>

const uint8_t MCP4561address = 0x2e;
int MCPWiper = 0;
//const uint8_t trackingSpeed = 4u;  // Maximum change of wiper: 4 steps/s, one step is approx. 0.1A
PAC1710 pac;

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
  if(MCPWiper < 0) MCPWiper = 0;
  else if(MCPWiper > 255) MCPWiper = 255;
}

/*int trackMPP(float voltage, float current) {
  static float oldV = voltage, oldI = current;
  if(voltage < 20.f) return -trackingSpeed; // Reduce current as much as possible
  if(current < .4f) return trackingSpeed; // Increase current as much as possible
  float dV = voltage - oldV;
  float mdIdV = (oldI - current) / dV;  // -dI/dV, always positive, expected to be < 2.5
  int command = floorf((mdIdV - current / voltage) * trackingSpeed);
  if(command > trackingSpeed) command = trackingSpeed;
  if(command != 0) {
    oldV = voltage;
    oldI = current;
  }
  return command;
}*/



void setup() {
  Wire.begin();
  setWiper(MCPWiper);
  //pac.SetSamplingTimesMs(20, 80);
  //pac.SetAveraging(PAC1710::AVG_NONE, PAC1710::AVG_8);
  pac.Init(PAC1710::SS_20MV, 2.0f);
  
  ST7565::begin(0x03);
  ST7565::display();
  delay(1000);
  ST7565::clear();

  ST7565::drawchar(0, 0, 'V');
  ST7565::drawchar(6, 0, ':');
  ST7565::drawchar(0, 1, 'A');
  ST7565::drawchar(6, 1, ':');
  ST7565::drawchar(0, 2, 'P');
  ST7565::drawchar(6, 2, ':');
  ST7565::drawchar(0, 3, 'W');
  ST7565::drawchar(6, 3, ':');
}

const float crashVoltage = 25.f, minVoltage = 30.9f, maxVoltage = 32.f;
constexpr unsigned long loopTimeUs = 100000;

void loop() {
  unsigned long startTime = micros();

  //pac.ReadOnce(PAC1710::READ_VOLTAGE);
  float volts = pac.GetVoltage(), amps = pac.GetCurrent(), power = pac.GetPower();
  if(volts < crashVoltage) setWiper(0);  // Voltage crashed, reset
  else if(volts < minVoltage) moveWiper(true);
  else if(volts > maxVoltage) moveWiper(false);

  int volti = volts * 10.f, ampi = amps * 100.f, powi = power;
  ampi = abs(ampi);

  ST7565::drawchar(12, 0, volti / 100 + 48);
  ST7565::drawchar(18, 0, (volti / 10) % 10 + 48);
  ST7565::drawchar(24, 0, '.');
  ST7565::drawchar(30, 0, volti % 10 + 48);

  ST7565::drawchar(12, 1, amps >= 0 ? '+' : '-');
  ST7565::drawchar(18, 1, ampi / 100 + 48);
  ST7565::drawchar(24, 1, '.');
  ST7565::drawchar(30, 1, (ampi / 10) % 10 + 48);
  ST7565::drawchar(36, 1, ampi % 10 + 48);

  ST7565::drawchar(12, 2, powi / 100 + 48);
  ST7565::drawchar(18, 2, (powi / 10) % 10 + 48);
  ST7565::drawchar(24, 2, powi % 10 + 48);

  ST7565::drawchar(12, 3, MCPWiper / 100 + 48);
  ST7565::drawchar(18, 3, (MCPWiper / 10) % 10 + 48);
  ST7565::drawchar(24, 3, MCPWiper % 10 + 48);

  ST7565::display();

  delayMicroseconds(loopTimeUs + startTime - micros()); // Beware of overflow!
  /*Serial.print("Power: ");
  Serial.print(power);
  Serial.print("W, current: ");
  Serial.print(current);
  Serial.print("A, voltage: ");
  Serial.print(voltage);
  Serial.print("V, wiper: ");
  Serial.println(MCPWiper);*/
}
