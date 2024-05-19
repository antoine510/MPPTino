#include <EEPROM.h>
#include <MCP4716.h>

#define USE_LCD 1
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint8_t MAGIC_NUMBER = 0x42;
constexpr uint16_t mppVoltage_dV = 320;  // 32V default MPP voltage
constexpr uint16_t minWakeVoltage_cV = 2800;
constexpr uint32_t minPowerMPPT_mw = 5000ul;
constexpr uint16_t maxVoltageDeltaMPP_cv = 50;
constexpr unsigned long stateUpdatePeriod = 1000ul;
constexpr unsigned long sleepCheckPeriod = 5000ul;
constexpr unsigned long wakeupMaxDuration = 1000ul;

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_TXEN = 2,       // PD2
  PIN_EN_LTC3813 = 3, // PD3
  PIN_EN_TOP_MOS = 4, // PD4
  PIN_LED = 13
};

enum CommandID : uint8_t {
  MAGIC = 0x0,
  READ_ALL = 0x1,
  SET_MPP_MANUAL_DV = 0x2,	// Manually set MPP voltage in decivolts
  SET_MPP_AUTO = 0x3,
  ENABLE_OUTPUT = 0x4,
  DISABLE_OUTPUT = 0x5
};

struct SerialData {
  uint16_t vin_cv;
  uint16_t vout_dv;
  uint16_t iout_ca;
  uint16_t eout_j;
};
SerialData sdata;
uint32_t eout_mj = 0;

MCP4716 dac;
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return mppv_dv * 17 / 10 + 196; }
constexpr uint16_t DACToVoltage_dv(uint16_t dacVal) { return (dacVal - 196) * 10 / 17; }
uint16_t dacValue = VoltageToDAC(mppVoltage_dV);

constexpr uint16_t dacMin = VoltageToDAC(280), dacMax = VoltageToDAC(340);

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
  if(increase) ++dacValue; else dacValue--;
  bool saturated = true;
  if(dacValue < dacMin) dacValue = dacMin;
  else if(dacValue > dacMax) dacValue = dacMax;
  else saturated = false;
  dac.SetValue(dacValue);
  return saturated;
}

bool isSleeping = true;
void goToSleep() {
  digitalWrite(PIN_EN_TOP_MOS, LOW);
  digitalWrite(PIN_EN_LTC3813, LOW);
  digitalWrite(PIN_LED, LOW);
  isSleeping = true;
}

void wakeup() {
  digitalWrite(PIN_EN_LTC3813, HIGH);
  unsigned long stopTrying = millis() + wakeupMaxDuration;
  while(millis() < stopTrying) {
    if(analogRead(PIN_IOUT) > 10) {
      digitalWrite(PIN_EN_TOP_MOS, HIGH); // Positive power, enable synchronous rectification
      digitalWrite(PIN_LED, HIGH);
      isSleeping = false;
      return;
    }
  }
  digitalWrite(PIN_EN_LTC3813, LOW);  // Wakeup failed
}

void setup() {
  Serial.begin(9600);

  digitalWrite(PIN_TXEN, LOW);
  pinMode(PIN_TXEN, OUTPUT);

  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_LED, OUTPUT);

#if USE_LCD
  setupLCD();
#endif

  dac.SetValue(dacValue); // Initialize DAC to startup value

  goToSleep();
  pinMode(PIN_EN_LTC3813, OUTPUT);
  pinMode(PIN_EN_TOP_MOS, OUTPUT);
}

void SendRS485(const uint8_t* data, size_t len) {
  PORTD |= 0x04;
  Serial.write(data, len);
  Serial.flush();
  PORTD &= 0xfb;
}

uint16_t manualMPP = 0; // 0 for automatic
bool forceDisableOutput = false;
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
    forceDisableOutput = false;
    break;
  case DISABLE_OUTPUT:
    forceDisableOutput = true;
    goToSleep();
    break;
  }
}


unsigned long nextWake = 0;
void checkSleep() {
  if(forceDisableOutput || analogRead(PIN_IOUT) != 0) return; // Power being produced
  if(isSleeping) {
    if(vinTransform_cV(analogRead(PIN_VIN)) > minWakeVoltage_cV) wakeup();
  } else {
    goToSleep();
  }
}

void updateState() {
  static bool increaseMPPV = true;
  static uint32_t lastPout_mw = 0;

  sdata.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  sdata.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  sdata.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  if(sdata.iout_ca > 0) sdata.iout_ca += 5; // Compensates for input offset voltage of sense amplifier

  static_assert(stateUpdatePeriod == 1000ul);
  uint32_t pout_mw = (uint32_t)sdata.vout_dv * sdata.iout_ca;
  eout_mj += pout_mw;

  if(manualMPP) SetMPPVoltage(manualMPP);
  else if(pout_mw > minPowerMPPT_mw) {
    if(abs(sdata.vin_cv - DACToVoltage_dv(dacValue) * 10) < maxVoltageDeltaMPP_cv) {  // Input voltage is linked to DAC setting
      bool saturated = NudgeMPP(increaseMPPV);
      if(saturated || pout_mw < lastPout_mw) increaseMPPV = !increaseMPPV;
      lastPout_mw = pout_mw;
    }
  } else SetMPPVoltage(mppVoltage_dV);

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
  static unsigned long nextStateUpdate = 0, nextSleepCheck = 0;
  if(millis() > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
  }
  if(millis() > nextSleepCheck) {
    nextSleepCheck += sleepCheckPeriod;
    checkSleep();
  }

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

  uint16_t pout = (uint32_t)sdata.iout_ca * sdata.vout_dv / 1000;
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
