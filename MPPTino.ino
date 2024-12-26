#include <EEPROM.h>
#include <MCP4716.h>
#include <avr/wdt.h>

#define USE_LCD 0
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint16_t maxMPPVoltage_dv = 350;
constexpr uint16_t minMPPVoltage_dv = 280;
constexpr uint16_t minPowerMPPT_dw = 50;
constexpr uint16_t wakeupVoltage_cv = 3300;
constexpr uint16_t dacStep = 4;
constexpr unsigned long stateUpdatePeriod = 500;
constexpr unsigned long stateAveragingPeriod = 60000;
constexpr unsigned long wakeupCheckDuration = 500;
constexpr unsigned long wakeupCheckWDTP = 7;  // wakeup check period in multiples of WDR period (8s)

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_TXEN = 2,         // PD2
  PIN_EN_LTC3813 = 3,   // PD3
  PIN_LED = 13          // PB5
};

enum CommandID : uint8_t {
  MAGIC = 0x0,
  READ_ALL = 0x1,
  SET_MPP_MANUAL_DV = 0x2,
  SET_MPP_AUTO = 0x3,
  ENABLE_OUTPUT = 0x4,
  DISABLE_OUTPUT = 0x5
};

struct PowerPoint {
  uint16_t vin_cv;
  uint16_t vout_dv;
  uint16_t iout_ca;
  uint16_t pout_dw;
};
PowerPoint power{};

struct SummedPowerPoint {
  uint32_t vin_cv;
  uint32_t vout_dv;
  uint32_t iout_ca;
  uint32_t pout_dw;
};
PowerPoint operator/(SummedPowerPoint sum, uint8_t s) {
  return {(uint16_t)(sum.vin_cv / s), (uint16_t)(sum.vout_dv / s), (uint16_t)(sum.iout_ca / s), (uint16_t)(sum.pout_dw / s)};
}
SummedPowerPoint operator+(SummedPowerPoint sum, PowerPoint a) {
  return {sum.vin_cv + a.vin_cv, sum.vout_dv + a.vout_dv, sum.iout_ca + a.iout_ca, sum.pout_dw + a.pout_dw};
}
SummedPowerPoint summedPower{};
uint8_t numSamples = 0;

PowerPoint averagePower{};
void updateAverage() {
  averagePower = summedPower / numSamples;
  memset((uint8_t*)(&summedPower), 0, sizeof(summedPower));
  numSamples = 0;
}

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

void SendRS485(const uint8_t* data, size_t len, bool withCRC = true);
inline void enableADC() { ADCSRA |= _BV(ADEN); }
inline void disableADC() { ADCSRA &= ~_BV(ADEN); }

bool sleeping = false;
volatile uint8_t wdtWakeups = 0;  // 8s each
void goToSleep() {
  SetMPPVoltage(minMPPVoltage_dv);
  PORTD &= ~_BV(PORTD3);
  PORTB &= ~_BV(PORTB5);
  disableADC();

  sleeping = true;
  wdtWakeups = 0;
}

void resumeSleep() {
  SMCR = _BV(SM1) | _BV(SE);
  sleep_cpu();
  SMCR = 0;
}

void wakeup() {
  if(vinTransform_cV(analogRead(PIN_VIN)) > wakeupVoltage_cv) {
    PORTD |= _BV(PORTD3);
    PORTB |= _BV(PORTB5);

    delay(wakeupCheckDuration);
    if(analogRead(PIN_IOUT) > 0) {
      sleeping = false;
    } else {
      PORTD &= ~_BV(PORTD3);
      PORTB &= ~_BV(PORTB5);
    }
  }
}

void readSensors() {
  power.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  power.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  power.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  power.pout_dw = (uint32_t)power.iout_ca * power.vout_dv / 100;
  summedPower = summedPower + power;
  numSamples++;
}

ISR(WDT_vect) {
  wdtWakeups++;
}

extern "C" void __attribute__((naked, used, section (".init3"))) init3() {
  DDRB = _BV(DDRB5);
  DDRD = _BV(DDRD2) | _BV(DDRD3);
}

void setup() {
  Serial.begin(9600);

  // Enable wakeup on serial byte complete, RXCIE is set by Serial.begin
  UCSR0D = _BV(SFDE);
  // Enable watchdog to wakeup every 8s
  cli();
  wdt_reset();
  WDTCSR |= _BV(WDCE) | _BV(WDE); // Enable WDT changes
  WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Setup watchdog wakeup in 8s
  sei();

#if USE_LCD
  setupLCD();
#endif

  readSensors();
  updateAverage();

  goToSleep();
}

void SendRS485(const uint8_t* data, size_t len, bool withCRC) {
  const uint8_t msg_crc = crc(data, len);
  PORTD |= _BV(PORTD2);
  Serial.write(data, len);
  if(withCRC) Serial.write(msg_crc);
  Serial.flush();
  PORTD &= ~_BV(PORTD2);
}

uint16_t manualMPP_dv = 0; // 0 for automatic
bool forceDisableOutput = false;
void runCommand(CommandID command) {
  constexpr uint8_t MAGIC_NUMBER = 0x42;
  switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER), false);
    break;
  case READ_ALL:
    SendRS485((uint8_t*)(&averagePower), sizeof(averagePower));
    break;
  case SET_MPP_MANUAL_DV:
    Serial.readBytes((uint8_t*)&manualMPP_dv, sizeof(manualMPP_dv));
    SetMPPVoltage(manualMPP_dv);
    break;
  case SET_MPP_AUTO:
    manualMPP_dv = 0;
    break;
  case ENABLE_OUTPUT:
    forceDisableOutput = false;
    break;
  case DISABLE_OUTPUT:
    forceDisableOutput = true;
    if(!sleeping) goToSleep();
    break;
  }
}

void updateState() {
  static bool increaseMPPV = true;
  static uint16_t lastPower_dw = 0;

  readSensors();

  if(power.iout_ca == 0 && !sleeping) goToSleep();

  if(!manualMPP_dv) {
    if(power.pout_dw > minPowerMPPT_dw) {
      if(power.pout_dw < lastPower_dw) increaseMPPV = !increaseMPPV;
      if(NudgeMPP(increaseMPPV)) increaseMPPV = !increaseMPPV;
    } else {
      SetMPPVoltage(minMPPVoltage_dv);
    }
  }

  lastPower_dw = power.pout_dw;

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
  static unsigned long nextStateUpdate = 0;
  if(!sleeping) {
    if(millis() > nextStateUpdate) {
      nextStateUpdate += stateUpdatePeriod;
      updateState();
      if(numSamples >= (stateAveragingPeriod / stateUpdatePeriod)) updateAverage();
    }
  } else {
    resumeSleep();
    if(wdtWakeups >= wakeupCheckWDTP) {
      enableADC();
      if(!forceDisableOutput) wakeup();
      updateState();
      if(sleeping) {  // Did not wakeup
        disableADC();
        wdtWakeups = 0;
      }
      updateAverage();
    }
  }

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
  }
}

uint8_t crc(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t b = 0; b < len; ++b) {
    crc ^= data[b];
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;	// SMBUS CRC8
      else
        crc = crc << 1;
    }
  }
  return crc;
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
