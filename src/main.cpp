#include <Arduino.h>
#include <Wire.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <INA226.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*********************************************************************
 * ESP32 Dual Battery Monitor with NMEA2000 Output
 * Baseline Stable Release: FW v0.8.0
 * 
 * ⚙️ CONFIGURATION INSTRUCTIONS
 * 1) DEBUGGING: set DEBUG 1/0
 * 2) CAPACITY (Ah): ENGINE/HOUSE
 * 3) NOMINAL_VOLTAGE: 12 or 24
 * 4) BATTERY_CHEMISTRY: CHEMISTRY_FLA/AGM/GEL/LFP
 * 5) SHUNT: 200A/50mV → SHUNT_CURRENT_LSB 0.00625 & SHUNT_CAL 3277
 * 6) 2-POINT CAL: edit vRaw1/vCal1/vRaw2/vCal2 and i* in setupBank()
 * 7) DS18B20: GPIO 4 (index 0=Engine, 1=House)
 * 8) LED: GPIO 2 blinks on each PGN sent
 * 9) PGNs: 127508 & 127506 every 1s; 127513 at startup + every 30s
 *********************************************************************
 * 🔧 CALIBRATION PROCEDURE (2-point)
 * - Measure "raw" (from your display/logger) vs "true" (meter) at two points
 * - Fill vRaw1/vCal1/vRaw2/vCal2 and iRaw1/iCal1/iRaw2/iCal2 per bank
 *********************************************************************/

// ========================
//   Firmware Information
// ========================
#define FW_VERSION "v0.8.0"   // Baseline stable release

// === Default Configuration ===
#define DEBUG 0
#define ENGINE_CAPACITY_AH_NOM 100
#define HOUSE_CAPACITY_AH_NOM 100
#define NOMINAL_VOLTAGE 12

// Chemistry options
#define CHEMISTRY_FLA 0
#define CHEMISTRY_AGM 1
#define CHEMISTRY_GEL 2
#define CHEMISTRY_LFP 3
#define BATTERY_CHEMISTRY CHEMISTRY_FLA

// Shunt calibration for external 200A/50mV
#define SHUNT_CURRENT_LSB 0.00625
#define SHUNT_CAL 3277

// I2C addresses of INA226
#define ADDR_ENGINE 0x40
#define ADDR_HOUSE  0x41

// LED and OneWire
#define LED_PIN 2
#define ONEWIRE_BUS 4

// -------- Chemistry Table --------
struct ChemParams {
  float fullV;    // resting full detect
  float floatV;   // float detect (reserved for future)
  float emptyV;   // resting empty detect
  tN2kDCBatteryType n2kType;
  tN2kDCBatteryChemistry n2kChem;
};

const ChemParams chemTable[] = {
  {12.6, 13.2, 11.8, N2kDCbt_Flooded,    N2kDCbc_Flooded},    // FLA
  {12.8, 13.4, 12.0, N2kDCbt_AGM,        N2kDCbc_AGM},        // AGM
  {12.85,13.5, 12.0, N2kDCbt_GEL,        N2kDCbc_GEL},        // GEL
  {13.4, 13.6, 12.8, N2kDCbt_LithiumIon, N2kDCbc_LithiumIon}  // LiFePO4
};

const ChemParams &chem = chemTable[BATTERY_CHEMISTRY];
const float VOLT_SCALE = (NOMINAL_VOLTAGE==24)?2.0f:1.0f;

// -------- State --------
struct BankState {
  INA226 *ina;
  float v, i;           // calibrated voltage/current
  float tempC;          // DS18B20 reading
  float tEMA;           // smoothed temperature
  float soc;            // 0..100%
  float ah_nom;         // nominal (or learned) capacity
  float ah_rem;         // remaining Ah
  float cap_learn;      // learned capacity
  unsigned long lastUpdate;
  bool wasFull;
  // 2-point calibration
  float vRaw1, vCal1, vRaw2, vCal2;
  float iRaw1, iCal1, iRaw2, iCal2;
};

BankState engine, house;

OneWire oneWire(ONEWIRE_BUS);
DallasTemperature sensors(&oneWire);

// LED non-blocking
unsigned long ledOffAt=0;
void blinkLED(){ digitalWrite(LED_PIN,HIGH); ledOffAt=millis()+50; }
void handleLED(){ if(ledOffAt && millis()>=ledOffAt){ digitalWrite(LED_PIN,LOW); ledOffAt=0; } }

// Debug helper
void debugPGN(uint32_t pgn, uint8_t inst){
  #if DEBUG
    Serial.printf("Sent PGN %lu (instance %u)\n", (unsigned long)pgn, inst);
  #endif
}

// 2-point calibration function
float calibrate(float raw, float r1, float c1, float r2, float c2){
  if (r2==r1) return raw;
  float gain=(c2-c1)/(r2-r1);
  float offs=c1-gain*r1;
  return raw*gain+offs;
}

// Update per-bank
void updateBank(BankState &b, int tempIndex){
  unsigned long now=millis();
  float dt=(now - b.lastUpdate)/1000.0f;
  if (dt <= 0) dt = 0.001f;
  b.lastUpdate=now;

  // Raw readings
  float vRaw = b.ina->getBusVoltage();
  float iRaw = b.ina->getCurrent();

  // Apply calibration
  b.v = calibrate(vRaw, b.vRaw1,b.vCal1,b.vRaw2,b.vCal2) * VOLT_SCALE;
  b.i = calibrate(iRaw, b.iRaw1,b.iCal1,b.iRaw2,b.iCal2);

  // Temp read + EMA
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(tempIndex);
  if (!isnan(t)) {
    if (b.tEMA==0) b.tEMA=t;
    else b.tEMA = 0.9f*b.tEMA + 0.1f*t;
    b.tempC=t;
  }

  // Ah integrate (discharge is positive current)
  float dAh = b.i * dt / 3600.0f;
  b.ah_rem -= dAh;
  if (b.ah_rem < 0) b.ah_rem = 0;
  if (b.ah_rem > b.ah_nom) b.ah_rem = b.ah_nom;

  // SoC
  b.soc = (b.ah_rem / b.ah_nom) * 100.0f;

  // Full-charge detection (resting voltage and low charge current)
  if (b.v >= chem.fullV*VOLT_SCALE && fabs(b.i) < 0.02f*b.ah_nom) {
    b.ah_rem = b.ah_nom;
    b.soc = 100;
    if (!b.wasFull) {
      // capacity learning nudge towards current nominal
      b.cap_learn = 0.95f*b.cap_learn + 0.05f*b.ah_nom;
      b.ah_nom = b.cap_learn;
      b.wasFull = true;
    }
  } else {
    b.wasFull = false;
  }

  // Resting empty detection
  if (fabs(b.i) < 0.01f*b.ah_nom && b.v <= chem.emptyV*VOLT_SCALE) {
    b.ah_rem = 0;
    b.soc = 0;
  }
}

// ---- N2K Senders ----
void sendPGN127508(uint8_t inst, const BankState &b){
  tN2kMsg msg;
  // Voltage V, Current A, Temp C, SOC %, Ripple (not measured → NA)
  SetN2kPGN127508(msg, inst, b.v, b.i, b.tempC, b.soc, N2kDoubleNA);
  NMEA2000.SendMsg(msg);
  blinkLED(); debugPGN(127508, inst);
}

void sendPGN127506(uint8_t inst, const BankState &b){
  tN2kMsg msg;
  // instance, voltage, current, capacity remaining (Ah), time remaining (NA)
  SetN2kPGN127506(msg, inst, b.v, b.i, b.ah_rem, N2kDoubleNA);
  NMEA2000.SendMsg(msg);
  blinkLED(); debugPGN(127506, inst);
}

void sendPGN127513(uint8_t inst, const BankState &b){
  tN2kMsg msg;
  // Battery Configuration per common helper signature
  SetN2kPGN127513(msg,
                  inst,
                  chem.n2kType,
                  NOMINAL_VOLTAGE,
                  chem.n2kChem,
                  (uint16_t)round(b.ah_nom),
                  (int8_t)(isnan(b.tempC)?N2kInt8NA:round(b.tempC)),
                  105,   // Peukert*100 (approx; adjust per chemistry if desired)
                  95);   // Charge eff*100 (approx)
  NMEA2000.SendMsg(msg);
  blinkLED(); debugPGN(127513, inst);
}

// ---- Setup helpers ----
void setupBank(BankState &b, uint8_t addr, float capAh){
  b.ina = new INA226(addr);
  b.ina->begin();
  b.ina->setCalibration(SHUNT_CURRENT_LSB, SHUNT_CAL);
  b.v=b.i=0;
  b.tempC=NAN; b.tEMA=0;
  b.ah_nom=capAh; b.ah_rem=capAh; b.soc=100;
  b.cap_learn=capAh;
  b.lastUpdate=millis();
  b.wasFull=false;
  // identity calibration defaults
  b.vRaw1=0; b.vCal1=0; b.vRaw2=1; b.vCal2=1;
  b.iRaw1=0; b.iCal1=0; b.iRaw2=1; b.iCal2=1;
}

unsigned long lastFast=0, last513=0;

void setup(){
  #if DEBUG
    Serial.begin(115200);
    delay(1000);
    Serial.printf("ESP32 NMEA2000 Battery Monitor starting... FW %s\n", FW_VERSION);
  #endif
  pinMode(LED_PIN,OUTPUT);
  Wire.begin();

  setupBank(engine, ADDR_ENGINE, ENGINE_CAPACITY_AH_NOM);
  setupBank(house,  ADDR_HOUSE,  HOUSE_CAPACITY_AH_NOM);

  sensors.begin();

  // Product info includes firmware version for easy identification on the bus
  NMEA2000.SetProductInformation("12345678",
                                 100,
                                 "ESP32 BattMon",
                                 FW_VERSION,   // Model version field carries firmware version
                                 "1.0");
  NMEA2000.SetDeviceInformation(1, 140, 25, 2046); // (unique id, function, class, manufacturer code)
  NMEA2000.EnableForward(false);
  NMEA2000.Open();

  // Send 127513 at startup
  sendPGN127513(0, engine);
  sendPGN127513(1, house);
  last513=millis();
}

void loop(){
  NMEA2000.ParseMessages();
  unsigned long now=millis();

  // Update banks
  updateBank(engine,0);
  updateBank(house,1);

  // Fast PGNs every 1s
  if (now - lastFast > 1000){
    sendPGN127508(0, engine);
    sendPGN127508(1, house);
    sendPGN127506(0, engine);
    sendPGN127506(1, house);
    lastFast = now;
  }

  // 127513 every 30s
  if (now - last513 >= 30000){
    sendPGN127513(0, engine);
    sendPGN127513(1, house);
    last513 = now;
  }

  handleLED();
}