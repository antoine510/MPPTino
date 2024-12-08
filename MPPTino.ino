#include <EEPROM.h>
#include <MCP4716.h>

#define USE_LCD 0
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint8_t MAGIC_NUMBER = 0x42;
constexpr uint16_t maxMPPVoltage_dv = 350;
constexpr uint16_t minMPPVoltage_dv = 280;
constexpr uint16_t minPowerMPPT_dw = 50;
constexpr uint16_t wakeupVoltage_cv = 3300;
constexpr uint16_t dacStep = 4;
constexpr unsigned long stateUpdatePeriod = 500;
constexpr unsigned long energyUpdatePeriod = 1000;
constexpr unsigned long wakeupCheckDuration = 500;
constexpr unsigned long wakeupCheckPeriod = 60000;

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_TXEN = 2,         // PD2
  PIN_EN_LTC3813 = 3,   // PD3
  PIN_LED = 13
};

enum CommandID : uint8_t {
  MAGIC = 0x0,
  READ_ALL = 0x1,
  SET_MPP_MANUAL_DV = 0x2,
  SET_MPP_AUTO = 0x3,
  ENABLE_OUTPUT = 0x4,
  DISABLE_OUTPUT = 0x5
};

struct SerialData {
  uint16_t vin_cv;
  uint16_t vout_dv;
  uint16_t iout_ca;
  uint16_t eout_j;
  uint8_t crc;
};
SerialData sdata;
uint32_t eout_mj = 0;
uint16_t power_dw = 0;

MCP4716 dac;
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv - 261) * 9; }
uint16_t dacValue = VoltageToDAC(minMPPVoltage_dv);

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

bool sleeping = true;
void goToSleep() {
  if(sleeping) return;
  digitalWrite(PIN_EN_LTC3813, LOW);
  digitalWrite(PIN_LED, LOW);
  SetMPPVoltage(minMPPVoltage_dv);
  sleeping = true;
}

void wakeup() {
  if(vinTransform_cV(analogRead(PIN_VIN)) < wakeupVoltage_cv) return;

  digitalWrite(PIN_EN_LTC3813, HIGH);
  digitalWrite(PIN_LED, HIGH);

  unsigned long wakeupDeadline = millis() + wakeupCheckDuration;
  while(millis() < wakeupDeadline) {
    if(analogRead(PIN_IOUT) > 0) {
      sleeping = false;
      return;
    }
  }

  digitalWrite(PIN_EN_LTC3813, LOW);
  digitalWrite(PIN_LED, LOW);
}

void readSensors() {
  sdata.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  sdata.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  sdata.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  if(sdata.iout_ca > 0) sdata.iout_ca += 5; // Compensates for input offset voltage of sense amplifier
}

void updateEnergy() {
  static_assert(energyUpdatePeriod == 1000ul);
  eout_mj += (uint32_t)sdata.vout_dv * sdata.iout_ca;
}

void setup() {
  digitalWrite(PIN_EN_LTC3813, LOW);
  pinMode(PIN_EN_LTC3813, OUTPUT);

  digitalWrite(PIN_TXEN, LOW);
  pinMode(PIN_TXEN, OUTPUT);

  dac.SetValue(dacValue); // Initialize DAC to startup value

  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(9600);

#if USE_LCD
  setupLCD();
#endif
}

void SendRS485(const uint8_t* data, size_t len) {
  PORTD |= 0x04;
  Serial.write(data, len);
  Serial.flush();
  PORTD &= 0xfb;
}

uint16_t manualMPP_dv = 0; // 0 for automatic
bool forceDisableOutput = false;
void runCommand(CommandID command) {
  switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER));
    break;
  case READ_ALL:
    sdata.eout_j = eout_mj / 1000;
    eout_mj = 0;
    sdata.crc = crc((uint8_t*)&sdata, sizeof(SerialData) - 1);
    SendRS485((uint8_t*)(&sdata), sizeof(SerialData));
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
    goToSleep();
    break;
  }
}

void updateState() {
  static bool increaseMPPV = true;

  readSensors();

  if(sdata.iout_ca == 0) goToSleep();

  uint16_t lastPower_dw = power_dw;
  power_dw = (uint32_t)sdata.vout_dv * sdata.iout_ca / 100;

  if(!manualMPP_dv) {
    if(power_dw > minPowerMPPT_dw) {
      if(power_dw < lastPower_dw) increaseMPPV = !increaseMPPV;
      if(NudgeMPP(increaseMPPV)) increaseMPPV = !increaseMPPV;
    } else {
      SetMPPVoltage(minMPPVoltage_dv);
    }
  }

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
  static unsigned long nextStateUpdate = 0, nextEnergyUpdate = 0, nextWakeupCheck = 0;
  auto now = millis();
  if(now > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
  }
  if(now > nextEnergyUpdate) {
    nextEnergyUpdate += energyUpdatePeriod;
    updateEnergy();
  }
  if(now > nextWakeupCheck) {
    nextWakeupCheck += wakeupCheckPeriod;
    if(!forceDisableOutput && sleeping) wakeup();
  }

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
  }
}

uint8_t crc(uint8_t *data, uint8_t len) {
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
