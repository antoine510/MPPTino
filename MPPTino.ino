#include <EEPROM.h>
#include <MCP4716.h>

#define USE_LCD 1
#if USE_LCD
#include <ST7565.h>
#endif

constexpr uint8_t MAGIC_NUMBER = 0x42;
constexpr uint16_t mppVoltage_dV = 320;
constexpr uint32_t minPowerMPPT_mw = 5000;
constexpr uint16_t maxVoltageDeltaMPP_cv = 100;
constexpr unsigned long stateUpdatePeriod = 500;
constexpr unsigned long energyUpdatePeriod = 1000;
constexpr uint16_t minWakeupVoltage_cv = 280;
constexpr unsigned long wakeupCheckDuration = 500;
constexpr unsigned long wakeupCheckPeriod = 10000;

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
};
SerialData sdata;
uint32_t eout_mj = 0;

class PowerTrendEstimator {
public:
  void update() {
    uint32_t pout_mw = (uint32_t)sdata.vout_dv * sdata.iout_ca;

    avgPout_mw = (pout_mw + avgSamples * avgPout_mw + ((avgSamples + 1) / 2)) / (avgSamples + 1);
    recentPout_mw = (pout_mw + recentSamples * recentPout_mw + ((recentSamples + 1) / 2)) / (recentSamples + 1);
  }

  bool decreasing() {
    bool res = (avgPout_mw - recentPout_mw) > powerDecreaseThreshold_mw;
    if(res) recentPout_mw = avgPout_mw;
    return res;
  }

  void reset() {
    avgPout_mw = 0;
    recentPout_mw = 0;
  }

  static constexpr uint32_t powerDecreaseThreshold_mw = 5000;
  static constexpr uint8_t avgSamples = 15, recentSamples = 7;
  uint32_t avgPout_mw = 0, recentPout_mw = 0;
};
PowerTrendEstimator powerTrend;

MCP4716 dac;
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv - 255) * 350 / 47; }
constexpr uint16_t DACToVoltage_dv(uint16_t dacVal) { return dacVal * 47 / 350 + 255; }
uint16_t dacValue = VoltageToDAC(mppVoltage_dV);

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
  if(increase) {
    if(dacValue < 1023) ++dacValue;
    else return true;
  } else {
    if(dacValue > 0) dacValue--;
    else return true;
  }
  dac.SetValue(dacValue);
  return false;
}

bool sleeping = true;
void goToSleep() {
  if(sleeping) return;
  powerTrend.reset();

  digitalWrite(PIN_EN_LTC3813, LOW);
  digitalWrite(PIN_LED, LOW);
  sleeping = true;
}

void wakeup() {
  if(vinTransform_cV(analogRead(PIN_VIN)) < minWakeupVoltage_cv) return;

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

  Serial.begin(9600);

  digitalWrite(PIN_TXEN, LOW);
  pinMode(PIN_TXEN, OUTPUT);

  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_LED, OUTPUT);

#if USE_LCD
  setupLCD();
#endif

  dac.SetValue(dacValue); // Initialize DAC to startup value
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
    digitalWrite(PIN_LED, HIGH);
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

void updateState() {
  static bool increaseMPPV = true, blinkOn = false;

  readSensors();
  powerTrend.update();

  if(sdata.iout_ca == 0) goToSleep();

  if(!manualMPP) {
    if(powerTrend.avgPout_mw > minPowerMPPT_mw) {
      // Check if input voltage is controlled by DAC setting
      if(abs(sdata.vin_cv - DACToVoltage_dv(dacValue) * 10) < maxVoltageDeltaMPP_cv) {
        bool saturated = NudgeMPP(increaseMPPV);
        if(saturated || powerTrend.decreasing()) increaseMPPV = !increaseMPPV;
        digitalWrite(PIN_LED, blinkOn);
        blinkOn = !blinkOn;
      } else {
        digitalWrite(PIN_LED, HIGH);
      }
    } else {
      SetMPPVoltage(mppVoltage_dV);
      digitalWrite(PIN_LED, HIGH);
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
  if(!forceDisableOutput && now > nextWakeupCheck) {
    nextWakeupCheck = millis() + wakeupCheckPeriod;
    wakeup();
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
