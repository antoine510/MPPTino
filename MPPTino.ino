#include <MCP4716.h>

#define USE_LCD 0
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint16_t MPPVoltage_dv = 280;
constexpr unsigned long stateUpdatePeriod = 500;

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_TXEN = 2,         // PD2
  PIN_EN_LTC3813 = 3,   // PD3
  PIN_LED = 13          // PB5
};

struct PowerPoint {
  uint16_t vin_cv;
  uint16_t vout_dv;
  uint16_t iout_ca;
  uint16_t pout_dw;
};
PowerPoint power{};

MCP4716 dac;
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv - 261) * 9; }
uint16_t dacValue = 0;

constexpr uint16_t vinTransform_cV(uint16_t count) { return count * 50 / 11; }
constexpr uint16_t voutTransform_dV(uint16_t count) { return count * 39 / 38; }
constexpr uint16_t ioutTransform_cA(uint16_t count) { return count ? count * 8 / 5 + 5 : 0; }

void SetMPPVoltage(uint16_t mppv_dv) {
  auto newDACValue = VoltageToDAC(mppv_dv);
  if(newDACValue == dacValue) return;
  dacValue = newDACValue;
  dac.SetValue(dacValue);
}

void readSensors() {
  power.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  power.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  power.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  power.pout_dw = (uint32_t)power.iout_ca * power.vout_dv / 100;
}

extern "C" void __attribute__((naked, used, section (".init3"))) init3() {
  DDRB = _BV(DDRB5);
  DDRD = _BV(DDRD2) | _BV(DDRD3);
}

void setup() {
  PORTD |= _BV(PORTD3);
  PORTB |= _BV(PORTB5);

  SetMPPVoltage(MPPVoltage_dv);

#if USE_LCD
  setupLCD();
#endif
}

void updateState() {
  readSensors();

#if USE_LCD
  updateLCD();
#endif
}


void loop() {
  static unsigned long nextStateUpdate = 0;
  if(millis() > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
  }
}

#if USE_LCD
void setupLCD() {
  ST7565::begin(0);
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
  auto vin = power.vin_cv / 10;
  ST7565::drawchar_aligned(3, 0, vin % 10 + 48);
  vin /= 10;
  ST7565::drawchar_aligned(1, 0, vin % 10 + 48);
  vin /= 10;
  ST7565::drawchar_aligned(0, 0, vin ? vin + 48 : 0);

  auto vout = power.vout_dv;
  ST7565::drawchar_aligned(3, 2, vout % 10 + 48);
  vout /= 10;
  ST7565::drawchar_aligned(1, 2, vout % 10 + 48);
  vout /= 10;
  ST7565::drawchar_aligned(0, 2, vout ? vout + 48 : 0);

  auto iout = power.iout_ca;
  ST7565::drawchar_aligned(4, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(3, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(1, 4, iout % 10 + 48);
  iout /= 10;
  ST7565::drawchar_aligned(0, 4, iout ? iout + 48 : 0);

  uint16_t pout = power.pout_dw / 10;
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
