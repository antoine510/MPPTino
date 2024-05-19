#include <EEPROM.h>
#include <MCP4716.h>

#define USE_LCD 1
#if USE_LCD
#include <ST7565.h>
#endif

static constexpr uint8_t MAGIC_NUMBER = 0x42;
static constexpr uint16_t mppVoltage_dV = 320;  // 32V default MPP voltage
static constexpr unsigned long stateUpdatePeriod = 1000ul;

enum Pins {
  PIN_VIN = A1,
  PIN_VOUT = A2,
  PIN_IOUT = A3,
  PIN_TXEN = 2,       // PD2
  PIN_EN_LTC3813 = 9  // PB1
};

enum CommandID : uint8_t {
  MAGIC = 0x0,
  READ_ALL = 0x1,
  SET_MPP_MANUAL_DV = 0x2,	// Manually set MPP voltage in decivolts
  SET_MPP_AUTO = 0x3
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
constexpr uint16_t VoltageToDAC(uint16_t mppv_dv) { return (mppv_dv * 17) / 10 + 196; }
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

void setup() {
  Serial.begin(9600);

  digitalWrite(PIN_TXEN, LOW);
  pinMode(PIN_TXEN, OUTPUT);

#if USE_LCD
  setupLCD();
#endif

  dac.SetValue(dacValue); // Initialize DAC to startup value

  digitalWrite(PIN_EN_LTC3813, HIGH);
  pinMode(PIN_EN_LTC3813, OUTPUT);
}

void SendRS485(const uint8_t* data, size_t len) {
  PORTD |= 0x04;
  Serial.write(data, len);
  Serial.flush();
  PORTD &= 0xfb;
}

uint16_t manualMPP = 0; // 0 for automatic
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
  }
}

void updateState() {
  sdata.vin_cv = vinTransform_cV(analogRead(PIN_VIN));
  sdata.vout_dv = voutTransform_dV(analogRead(PIN_VOUT));
  sdata.iout_ca = ioutTransform_cA(analogRead(PIN_IOUT));
  if(sdata.iout_ca > 0) sdata.iout_ca += 5; // Compensates for input offset voltage of sense amplifier

  static_assert(stateUpdatePeriod == 1000ul);
  eout_mj += (uint32_t)sdata.vout_dv * sdata.iout_ca;

  SetMPPVoltage(manualMPP ? manualMPP : mppVoltage_dV);

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
  if(millis() > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
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
