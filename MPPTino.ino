#include <MCP4716.h>

#define USE_LCD 1
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint8_t MAGIC_NUMBER = 0x42;
constexpr uint16_t resetMPPVoltage_dv = 300;
constexpr uint16_t maxMPPVoltage_dv = 340;
constexpr uint16_t minMPPVoltage_dv = 290;
constexpr uint16_t minPowerMPPT_dw = 50;
constexpr uint16_t dacStep = 3;
constexpr unsigned long stateUpdatePeriod = 500;

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_EN_LTC3813 = 9,   // PB1
  PIN_LED = 13
};

struct SerialData {
  uint16_t vin_cv;
  uint16_t vout_dv;
  uint16_t iout_ca;
};
SerialData sdata;
uint16_t power_dw = 0;

MCP4716 dac;
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv * 17) / 10 + 196; }
uint16_t dacValue = VoltageToDAC(resetMPPVoltage_dv);

constexpr uint16_t vinTransform_cV(uint16_t count) { return count * 50 / 11; }
constexpr uint16_t voutTransform_dV(uint16_t count) { return count * 39 / 38; }
constexpr uint16_t ioutTransform_cA(uint16_t count) { return count * 8 / 5; }

void SetMPPVoltage(uint16_t mppv_dv) {
  auto newDACValue = VoltageToDAC(mppv_dv);
  if(newDACValue == dacValue) return;
  dacValue = newDACValue;
  dac.SetValue(dacValue);
}

bool NudgeMPP(bool increase) {
  static constexpr uint16_t dacMin = VoltageToDAC(minMPPVoltage_dv), dacMax = VoltageToDAC(maxMPPVoltage_dv);
  bool saturation = false;
  if(increase) {
    if(dacValue < dacMax - dacStep) dacValue += dacStep;
    else { dacValue = dacMax; saturation = true; }
  } else {
    if(dacValue > dacMin + dacStep) dacValue -= dacStep;
    else { dacValue = dacMin; saturation = true; }
  }
  dac.SetValue(dacValue);
  return saturation;
}

void readSensors() {
  sdata.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  sdata.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  sdata.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  if(sdata.iout_ca > 0) sdata.iout_ca += 5; // Compensates for input offset voltage of sense amplifier
}

void setup() {
  digitalWrite(PIN_EN_LTC3813, HIGH);
  pinMode(PIN_EN_LTC3813, OUTPUT);

  dac.SetValue(dacValue); // Initialize DAC to startup value

  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_LED, OUTPUT);

#if USE_LCD
  setupLCD();
#endif
}

void updateState() {
  static bool increaseMPPV = true;

  readSensors();

  uint16_t lastPower_dw = power_dw;
  power_dw = (uint32_t)sdata.vout_dv * sdata.iout_ca / 100;

  if(power_dw > minPowerMPPT_dw) {
    if(power_dw < lastPower_dw) increaseMPPV = !increaseMPPV;
    if(NudgeMPP(increaseMPPV)) increaseMPPV = !increaseMPPV;
  } else {
    SetMPPVoltage(resetMPPVoltage_dv);
  }

#if USE_LCD
  updateLCD();
#endif
}

void loop() {
  static unsigned long nextStateUpdate = 0;
  auto now = millis();
  if(now > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
  }
}

#if USE_LCD
void setupLCD() {
  ST7565::begin(0x02);
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
  auto vin = sdata.vin_cv / 10;
  ST7565::drawchar_aligned(3, 0, vin % 10 + 48);
  vin /= 10;
  ST7565::drawchar_aligned(1, 0, vin % 10 + 48);
  vin /= 10;
  ST7565::drawchar_aligned(0, 0, vin ? vin + 48 : 0);

  auto vout = sdata.vout_dv;
  ST7565::drawchar_aligned(3, 2, vout % 10 + 48);
  vout /= 10;
  ST7565::drawchar_aligned(1, 2, vout % 10 + 48);
  vout /= 10;
  ST7565::drawchar_aligned(0, 2, vout ? vout + 48 : 0);

  auto iout = sdata.iout_ca;
  ST7565::drawchar_aligned(4, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(3, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(1, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(0, 4, iout ? iout + 48 : 0);

  uint16_t pout = power_dw / 10;
  ST7565::drawchar_aligned(3, 6, pout % 10 + 48);
  pout /= 10;
  ST7565::drawchar_aligned(2, 6, pout ? pout % 10 + 48 : 0);
  pout /= 10;
  ST7565::drawchar_aligned(1, 6, pout ? pout % 10 + 48 : 0);
  pout /= 10;
  ST7565::drawchar_aligned(0, 6, pout ? pout + 48 : 0);

  ST7565::display();
}
#endif
