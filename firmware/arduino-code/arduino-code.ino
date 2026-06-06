/*
  PH-4502C + TDS + ADS1115 + I2C LCD 16x2
  Board: Arduino Nano ESP32 (also works on classic Nano with 5V LCD)

  Wiring:
    ADS1115 + LCD (shared I2C):
      SDA -> A4
      SCL -> A5
      VDD -> 3V3 (Nano ESP32)
      GND -> GND

    ADS1115 analog:
      PH-4502C PO -> A0
      TDS AO       -> A1

    STATUS_OUT -> D7  (LOW = both ideal, HIGH = otherwise)

  Classifications:
    pH  6.5 - 7.5  -> Ideal pH
    pH  < 6.5      -> Low pH
    pH  > 7.5      -> High pH
    TDS 75 - 250   -> Ideal TDS
    TDS < 75       -> Low TDS
    TDS > 250      -> High TDS

  Libraries (Library Manager):
    - Adafruit ADS1X15
    - LiquidCrystal I2C (Frank de Brabander)

  LCD address: usually 0x27 or 0x3F (change LCD_ADDR below if blank)
*/

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// Change to 0x3F if your LCD scanner finds that address instead of 0x27
const uint8_t LCD_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ---------------- Pins / channels ----------------
const uint8_t PH_CH      = 0;
const uint8_t TDS_CH     = 1;
const uint8_t STATUS_PIN = 7;

// ---------------- ADC ----------------
const float ADS_LSB_V = 0.000125f;   // GAIN_ONE (+/-4.096V)

// ---------------- Sampling ----------------
const uint8_t MEDIAN_SAMPLES     = 15;
const uint16_t SAMPLE_DELAY_MS   = 20;
const uint16_t PRINT_INTERVAL_MS = 1000;
const uint16_t LCD_INTERVAL_MS   = 500;

// ---------------- pH limits ----------------
const float PH_IDEAL_MIN = 6.5f;
const float PH_IDEAL_MAX = 7.5f;
const float PH_MIN       = 0.0f;
const float PH_MAX       = 14.0f;

// ---------------- TDS limits ----------------
const float TDS_IDEAL_MIN = 75.0f;
const float TDS_IDEAL_MAX = 250.0f;

// ---------------- EEPROM ----------------
const int EEPROM_MAGIC_ADDR        = 0;
const int EEPROM_SLOPE_ADDR        = 4;
const int EEPROM_OFFSET_ADDR       = 8;
const int EEPROM_TDS_BASE_ADDR     = 12;
const int EEPROM_TDS_FACTOR_ADDR   = 16;
const uint32_t EEPROM_MAGIC        = 0x48544431;  // "HTD1"

// ---------------- Calibration ----------------
float calSlope       = -5.70f;
float calOffset      = 21.34f;
float tdsBaselinePpm = 0.0f;
float tdsCalFactor   = 1.00f;
float waterTempC     = 25.0f;

const uint8_t AVG_SAMPLES = 10;
float phBuffer[AVG_SAMPLES];
uint8_t phBufIndex = 0;
bool phBufFilled   = false;

bool hasCal7 = false, hasCal4 = false;
float v7 = 0.0f, v4 = 0.0f;

enum PhStatus  { PH_LOW, PH_IDEAL, PH_HIGH };
enum TdsStatus { TDS_LOW, TDS_IDEAL, TDS_HIGH };

// ---------------- Forward declarations ----------------
float clampPH(float p);
float readAdsVoltage(uint8_t ch);
float readMedianVoltage(uint8_t ch, uint8_t nSamples);
float computePHFromVoltage(float v);
float readPHOnceFiltered();
float readPHAveraged();
float computeTdsPpm(float measuredVoltage, float tempC);
float readTdsCompensated();
PhStatus  classifyPH(float ph);
TdsStatus classifyTDS(float ppm);
const char* phStatusStr(PhStatus s);
const char* tdsStatusStr(TdsStatus s);
void updateStatusOutput(bool bothIdeal);
void updateLcd(float ph, float tds, PhStatus phSt, TdsStatus tdsSt, bool bothIdeal);
void saveCalibration();
bool loadCalibration();
void saveTdsSettings();
void loadTdsSettings();
bool waitForStablePH(float &outV, float &outPH, uint16_t durationMs = 5000, uint16_t stepMs = 200);
void printHelp();
void processSerial();
void scanI2C();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  lcd.setCursor(0, 1);
  lcd.print("ADS1115 init");

  if (!ads.begin(0x48)) {
    Serial.println(F("ADS1115 not found at 0x48."));
    lcd.clear();
    lcd.print("ADS1115 error");
    lcd.setCursor(0, 1);
    lcd.print("Check I2C wiring");
    while (1) delay(100);
  }
  ads.setGain(GAIN_ONE);

  if (!loadCalibration()) {
    Serial.println(F("No pH calibration in EEPROM. Use CAL7, CAL4, SAVE."));
  }
  loadTdsSettings();

  float initPH = readPHOnceFiltered();
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) phBuffer[i] = initPH;
  phBufFilled = true;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("pH + TDS ready");
  lcd.setCursor(0, 1);
  lcd.print("Monitoring...");

  Serial.println(F("Merged pH + TDS + LCD monitor ready."));
  printHelp();
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t lastLcd   = 0;

  processSerial();

  float ph  = readPHAveraged();
  float tds = readTdsCompensated();
  PhStatus  phSt  = classifyPH(ph);
  TdsStatus tdsSt = classifyTDS(tds);
  bool bothIdeal  = (phSt == PH_IDEAL) && (tdsSt == TDS_IDEAL);

  updateStatusOutput(bothIdeal);

  if (millis() - lastLcd >= LCD_INTERVAL_MS) {
    lastLcd = millis();
    updateLcd(ph, tds, phSt, tdsSt, bothIdeal);
  }

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();

    float phV  = readMedianVoltage(PH_CH, MEDIAN_SAMPLES);
    float tdsV = readMedianVoltage(TDS_CH, MEDIAN_SAMPLES);

    Serial.print(F("pH=")); Serial.print(ph, 2);
    Serial.print(F(" (")); Serial.print(phStatusStr(phSt));
    Serial.print(F(") | Vph=")); Serial.print(phV, 3);

    Serial.print(F(" | TDS=")); Serial.print(tds, 1);
    Serial.print(F(" ppm (")); Serial.print(tdsStatusStr(tdsSt));
    Serial.print(F(") | Vtds=")); Serial.print(tdsV, 3);

    Serial.print(F(" | STATUS_OUT="));
    Serial.println(bothIdeal ? F("LOW (ideal)") : F("HIGH (not ideal)"));
  }
}

// Full names for Serial; short names for 16x2 LCD
const char* phStatusShort(PhStatus s) {
  switch (s) {
    case PH_LOW:   return "Low pH";
    case PH_IDEAL: return "Ideal pH";
    case PH_HIGH:  return "High pH";
  }
  return "?";
}

const char* tdsStatusShort(TdsStatus s) {
  switch (s) {
    case TDS_LOW:   return "Low TDS";
    case TDS_IDEAL: return "Ideal TDS";
    case TDS_HIGH:  return "High TDS";
  }
  return "?";
}

void updateLcd(float ph, float tds, PhStatus phSt, TdsStatus tdsSt, bool bothIdeal) {
  char line1[17];
  char line2[17];

  // Full classification text, trimmed to 16 chars per line
  snprintf(line1, sizeof(line1), "pH%.2f %s", ph, phStatusShort(phSt));
  snprintf(line2, sizeof(line2), "TDS%.0f %s", tds, tdsStatusShort(tdsSt));

  lcd.setCursor(0, 0);
  lcd.print(line1);
  // Pad remainder of line to clear stale characters
  for (int i = strlen(line1); i < 16; i++) lcd.print(' ');

  lcd.setCursor(0, 1);
  lcd.print(line2);
  for (int i = strlen(line2); i < 16; i++) lcd.print(' ');

  // Brief overall indicator in corner not possible on 2 lines with full text;
  // D7 STATUS pin and Serial carry OK/NOT OK. bothIdeal used for future blink, etc.
  (void)bothIdeal;
}

float clampPH(float p) {
  if (p < PH_MIN) return PH_MIN;
  if (p > PH_MAX) return PH_MAX;
  return p;
}

float readAdsVoltage(uint8_t ch) {
  int16_t raw = ads.readADC_SingleEnded(ch);
  return raw * ADS_LSB_V;
}

float readMedianVoltage(uint8_t ch, uint8_t nSamples) {
  if (nSamples < 3) nSamples = 3;
  if (nSamples > 31) nSamples = 31;

  float samples[31];
  for (uint8_t i = 0; i < nSamples; i++) {
    samples[i] = readAdsVoltage(ch);
    delay(SAMPLE_DELAY_MS);
  }

  for (uint8_t i = 1; i < nSamples; i++) {
    float key = samples[i];
    int8_t j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[nSamples / 2];
}

float computePHFromVoltage(float v) {
  return calSlope * v + calOffset;
}

float readPHOnceFiltered() {
  float v = readMedianVoltage(PH_CH, MEDIAN_SAMPLES);
  return clampPH(computePHFromVoltage(v));
}

float readPHAveraged() {
  float p = readPHOnceFiltered();
  phBuffer[phBufIndex] = p;
  phBufIndex = (phBufIndex + 1) % AVG_SAMPLES;
  if (phBufIndex == 0) phBufFilled = true;

  uint8_t count = phBufFilled ? AVG_SAMPLES : phBufIndex;
  if (count == 0) return p;

  float sum = 0.0f;
  for (uint8_t i = 0; i < count; i++) sum += phBuffer[i];
  return sum / count;
}

bool waitForStablePH(float &outV, float &outPH, uint16_t durationMs, uint16_t stepMs) {
  uint32_t start = millis();
  uint16_t count = 0;
  float sumV = 0.0f, sumPH = 0.0f;

  while (millis() - start < durationMs) {
    float v = readMedianVoltage(PH_CH, MEDIAN_SAMPLES);
    float p = clampPH(computePHFromVoltage(v));
    sumV += v;
    sumPH += p;
    count++;
    Serial.print(F("."));
    delay(stepMs);
  }
  Serial.println();
  if (count == 0) return false;
  outV = sumV / count;
  outPH = sumPH / count;
  return true;
}

float computeTdsPpm(float measuredVoltage, float tempC) {
  float compensationCoefficient = 1.0f + 0.02f * (tempC - 25.0f);
  float compensationVoltage = measuredVoltage / compensationCoefficient;

  float tds = (133.42f * compensationVoltage * compensationVoltage * compensationVoltage
             - 255.86f * compensationVoltage * compensationVoltage
             + 857.39f * compensationVoltage) * 0.5f;

  tds *= tdsCalFactor;
  if (tds < 0) tds = 0;
  return tds;
}

float readTdsCompensated() {
  float v = readMedianVoltage(TDS_CH, MEDIAN_SAMPLES);
  float raw = computeTdsPpm(v, waterTempC);
  float comp = raw - tdsBaselinePpm;
  if (comp < 0) comp = 0;
  return comp;
}

PhStatus classifyPH(float ph) {
  if (ph < PH_IDEAL_MIN) return PH_LOW;
  if (ph > PH_IDEAL_MAX) return PH_HIGH;
  return PH_IDEAL;
}

TdsStatus classifyTDS(float ppm) {
  if (ppm < TDS_IDEAL_MIN) return TDS_LOW;
  if (ppm > TDS_IDEAL_MAX) return TDS_HIGH;
  return TDS_IDEAL;
}

const char* phStatusStr(PhStatus s) {
  switch (s) {
    case PH_LOW:   return "Low pH";
    case PH_IDEAL: return "Ideal pH";
    case PH_HIGH:  return "High pH";
  }
  return "?";
}

const char* tdsStatusStr(TdsStatus s) {
  switch (s) {
    case TDS_LOW:   return "Low TDS";
    case TDS_IDEAL: return "Ideal TDS";
    case TDS_HIGH:  return "High TDS";
  }
  return "?";
}

void updateStatusOutput(bool bothIdeal) {
  digitalWrite(STATUS_PIN, bothIdeal ? LOW : HIGH);
}

void saveCalibration() {
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_SLOPE_ADDR, calSlope);
  EEPROM.put(EEPROM_OFFSET_ADDR, calOffset);
}

bool loadCalibration() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) return false;

  float s, o;
  EEPROM.get(EEPROM_SLOPE_ADDR, s);
  EEPROM.get(EEPROM_OFFSET_ADDR, o);

  if (isnan(s) || isnan(o) || isinf(s) || isinf(o)) return false;
  if (s > 0.0f || s < -20.0f) return false;
  if (o < -10.0f || o > 40.0f) return false;

  calSlope = s;
  calOffset = o;
  return true;
}

void saveTdsSettings() {
  EEPROM.put(EEPROM_TDS_BASE_ADDR, tdsBaselinePpm);
  EEPROM.put(EEPROM_TDS_FACTOR_ADDR, tdsCalFactor);
}

void loadTdsSettings() {
  EEPROM.get(EEPROM_TDS_BASE_ADDR, tdsBaselinePpm);
  EEPROM.get(EEPROM_TDS_FACTOR_ADDR, tdsCalFactor);
  if (isnan(tdsBaselinePpm) || isnan(tdsCalFactor)) {
    tdsBaselinePpm = 0.0f;
    tdsCalFactor = 1.00f;
  }
}

void scanI2C() {
  Serial.println(F("I2C scan:"));
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println(F("  No devices found"));
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  READ, HELP, SCAN"));
  Serial.println(F("  CAL / CAL7 / CAL4 / SAVE / SHOWCAL / RESETCAL  (pH)"));
  Serial.println(F("  ZERO          (TDS baseline in air/distilled)"));
  Serial.println(F("  TEMP 25.0     (water temp C)"));
  Serial.println(F("  FACTOR 1.00   (TDS scale factor)"));
}

void processSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "HELP" || upper == "?") {
    printHelp();
  } else if (upper == "SCAN") {
    scanI2C();
  } else if (upper == "READ") {
    float ph = readPHAveraged();
    float tds = readTdsCompensated();
    PhStatus phSt = classifyPH(ph);
    TdsStatus tdsSt = classifyTDS(tds);
    bool bothIdeal = (phSt == PH_IDEAL) && (tdsSt == TDS_IDEAL);
    updateStatusOutput(bothIdeal);
    updateLcd(ph, tds, phSt, tdsSt, bothIdeal);

    Serial.print(F("pH=")); Serial.print(ph, 2); Serial.print(F(" ")); Serial.println(phStatusStr(phSt));
    Serial.print(F("TDS=")); Serial.print(tds, 1); Serial.print(F(" ")); Serial.println(tdsStatusStr(tdsSt));
    Serial.println(bothIdeal ? F("STATUS_OUT=LOW") : F("STATUS_OUT=HIGH"));
  } else if (upper == "CAL") {
    hasCal7 = hasCal4 = false;
    Serial.println(F("Put probe in pH 7 -> CAL7, then pH 4 -> CAL4, then SAVE"));
  } else if (upper == "CAL7") {
    float p;
    if (waitForStablePH(v7, p)) { hasCal7 = true; Serial.print(F("V7=")); Serial.println(v7, 5); }
  } else if (upper == "CAL4") {
    float p;
    if (waitForStablePH(v4, p)) { hasCal4 = true; Serial.print(F("V4=")); Serial.println(v4, 5); }
  } else if (upper == "SAVE") {
    if (!hasCal7 || !hasCal4) { Serial.println(F("Need CAL7 and CAL4.")); return; }
    if (fabs(v4 - v7) < 0.005f) { Serial.println(F("Points too close.")); return; }
    calSlope  = (4.00f - 7.00f) / (v4 - v7);
    calOffset = 7.00f - calSlope * v7;
    saveCalibration();
    Serial.println(F("pH calibration saved."));
  } else if (upper == "SHOWCAL") {
    Serial.print(F("Slope=")); Serial.println(calSlope, 6);
    Serial.print(F("Offset=")); Serial.println(calOffset, 6);
    Serial.print(F("TDS baseline=")); Serial.println(tdsBaselinePpm, 1);
    Serial.print(F("TDS factor=")); Serial.println(tdsCalFactor, 3);
    Serial.print(F("TempC=")); Serial.println(waterTempC, 1);
  } else if (upper == "RESETCAL") {
    calSlope = -5.70f; calOffset = 21.34f;
    saveCalibration();
    Serial.println(F("pH cal reset to defaults."));
  } else if (upper == "ZERO") {
    float v = readMedianVoltage(TDS_CH, 21);
    tdsBaselinePpm = computeTdsPpm(v, waterTempC);
    saveTdsSettings();
    Serial.print(F("TDS baseline set to ")); Serial.print(tdsBaselinePpm, 1); Serial.println(F(" ppm"));
  } else if (upper.startsWith("TEMP ")) {
    waterTempC = line.substring(5).toFloat();
    Serial.print(F("Temp=")); Serial.println(waterTempC, 1);
  } else if (upper.startsWith("FACTOR ")) {
    tdsCalFactor = line.substring(7).toFloat();
    saveTdsSettings();
    Serial.print(F("Factor=")); Serial.println(tdsCalFactor, 3);
  } else {
    Serial.println(F("Unknown command. Type HELP"));
  }
}
