/*******************
 * ESP32 – Water Treatment Plant Control (Example Code)
 *
 *  - Reads sensors (flow, turbidity, pH, chlorine, level, pressure)
 *  - Controls pumps & chemical dosing via relay outputs
 *
 *  NOTE:
 *   - All thresholds, calibration factors and pin numbers
 *     MUST be adapted to your real hardware.
 *******************/

// ---------- Pin mapping ----------
// Analog inputs (use only ADC-capable pins on ESP32)
const int PIN_TURBIDITY_INTAKE      = 34;
const int PIN_TURBIDITY_AFTER_SED   = 35;
const int PIN_TURBIDITY_AFTER_FILT  = 32;
const int PIN_PH_SENSOR             = 33;
const int PIN_CL_SENSOR             = 25;   // chlorine residual
const int PIN_LEVEL_SENSOR          = 26;   // storage level (e.g. 0–100% as voltage)
const int PIN_PRESSURE_SENSOR       = 27;   // distribution pressure

// Digital inputs – flow sensors (hall effect with pulses)
const int PIN_FLOW_INTAKE           = 4;
const int PIN_FLOW_AFTER_FILT       = 5;
const int PIN_FLOW_DIST             = 18;

// Relay outputs (HIGH = relay ON)
const int PIN_RELAY_RAW_PUMP        = 12;
const int PIN_RELAY_BOOSTER_PUMP    = 13;
const int PIN_RELAY_PH_PUMP         = 14;
const int PIN_RELAY_ANTICORR_PUMP   = 15;
const int PIN_RELAY_CL_PUMP         = 2;

// ---------- Flow measurement variables ----------
volatile unsigned long pulsesIntake      = 0;
volatile unsigned long pulsesAfterFilt   = 0;
volatile unsigned long pulsesDist        = 0;

unsigned long lastFlowCalcMs = 0;
const unsigned long FLOW_INTERVAL_MS = 1000;   // 1 second

float flowIntake_Lmin     = 0.0;
float flowAfterFilt_Lmin  = 0.0;
float flowDist_Lmin       = 0.0;

// Adjust according to your sensor datasheet (pulses per liter)
const float PULSES_PER_LITER = 450.0;

// ---------- Control timing ----------
unsigned long lastControlMs = 0;
const unsigned long CONTROL_INTERVAL_MS = 1000; // 1 second

// ---------- Calibration factors (very rough examples) ----------
// ESP32 ADC: 0–4095 corresponds to 0–3.3 V

// Level sensor: convert ADC → percent (0–100%)
float levelPercentFromADC(int adc)
{
  // Example: 0.5 V (620) = 0%, 2.8 V (3470) = 100%
  const float adcMin = 620.0;
  const float adcMax = 3470.0;
  float pct = (adc - adcMin) * 100.0 / (adcMax - adcMin);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// Pressure sensor: convert ADC → bar
float pressureFromADC(int adc)
{
  // Example 0.5–4.5 V → 0–6 bar
  float voltage = adc * 3.3 / 4095.0;
  float press = (voltage - 0.5) * (6.0 / (4.0)); // 0.5–4.5 span
  if (press < 0) press = 0;
  return press;
}

// pH sensor: simple linear example, 0–14 pH
float pHFromADC(int adc)
{
  float voltage = adc * 3.3 / 4095.0;
  // Example: 0 V → pH 0, 3.0 V → pH 14 (totally fake, replace with calibration)
  float ph = voltage * (14.0 / 3.0);
  if (ph < 0) ph = 0;
  if (ph > 14) ph = 14;
  return ph;
}

// Chlorine residual sensor: 0–3.3 V → 0–2 mg/L (example)
float chlorineFromADC(int adc)
{
  float voltage = adc * 3.3 / 4095.0;
  float cl = voltage * (2.0 / 3.3);
  if (cl < 0) cl = 0;
  return cl;
}

// Turbidity: just return 0–100 NTU as an example
float turbidityFromADC(int adc)
{
  // 3.0 V (3720) = 0 NTU (very clear), 0.5 V (620) = 100 NTU (dirty)
  float turbidity = (3720 - adc) * 100.0 / (3720 - 620);
  if (turbidity < 0) turbidity = 0;
  if (turbidity > 100) turbidity = 100;
  return turbidity;
}

// ---------- Flow ISR handlers ----------
void IRAM_ATTR isrFlowIntake()
{
  pulsesIntake++;
}

void IRAM_ATTR isrFlowAfterFilt()
{
  pulsesAfterFilt++;
}

void IRAM_ATTR isrFlowDist()
{
  pulsesDist++;
}

// ---------- Helper: set relay state ----------
void setRelay(int pin, bool on)
{
  digitalWrite(pin, on ? HIGH : LOW);
}

// ---------- Setup ----------
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Water Treatment Control – starting...");

  // Configure analog inputs
  pinMode(PIN_TURBIDITY_INTAKE,    INPUT);
  pinMode(PIN_TURBIDITY_AFTER_SED, INPUT);
  pinMode(PIN_TURBIDITY_AFTER_FILT,INPUT);
  pinMode(PIN_PH_SENSOR,           INPUT);
  pinMode(PIN_CL_SENSOR,           INPUT);
  pinMode(PIN_LEVEL_SENSOR,        INPUT);
  pinMode(PIN_PRESSURE_SENSOR,     INPUT);

  // Configure flow sensor pins
  pinMode(PIN_FLOW_INTAKE,         INPUT_PULLUP);
  pinMode(PIN_FLOW_AFTER_FILT,     INPUT_PULLUP);
  pinMode(PIN_FLOW_DIST,           INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_INTAKE),     isrFlowIntake,    RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_AFTER_FILT), isrFlowAfterFilt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_DIST),       isrFlowDist,      RISING);

  // Configure relay outputs
  pinMode(PIN_RELAY_RAW_PUMP,      OUTPUT);
  pinMode(PIN_RELAY_BOOSTER_PUMP,  OUTPUT);
  pinMode(PIN_RELAY_PH_PUMP,       OUTPUT);
  pinMode(PIN_RELAY_ANTICORR_PUMP, OUTPUT);
  pinMode(PIN_RELAY_CL_PUMP,       OUTPUT);

  // All OFF to start
  setRelay(PIN_RELAY_RAW_PUMP,      false);
  setRelay(PIN_RELAY_BOOSTER_PUMP,  false);
  setRelay(PIN_RELAY_PH_PUMP,       false);
  setRelay(PIN_RELAY_ANTICORR_PUMP, false);
  setRelay(PIN_RELAY_CL_PUMP,       false);
}

// ---------- Main loop ----------
void loop()
{
  unsigned long now = millis();

  // 1) Calculate flows every FLOW_INTERVAL_MS
  if (now - lastFlowCalcMs >= FLOW_INTERVAL_MS)
  {
    noInterrupts();
    unsigned long pIntake    = pulsesIntake;
    unsigned long pAfterFilt = pulsesAfterFilt;
    unsigned long pDist      = pulsesDist;
    pulsesIntake    = 0;
    pulsesAfterFilt = 0;
    pulsesDist      = 0;
    interrupts();

    // pulses per second → L/min
    flowIntake_Lmin    = (pIntake    / PULSES_PER_LITER) * 60.0;
    flowAfterFilt_Lmin = (pAfterFilt / PULSES_PER_LITER) * 60.0;
    flowDist_Lmin      = (pDist      / PULSES_PER_LITER) * 60.0;

    lastFlowCalcMs = now;
  }

  // 2) Control & monitoring every CONTROL_INTERVAL_MS
  if (now - lastControlMs >= CONTROL_INTERVAL_MS)
  {
    lastControlMs = now;

    // Read analog sensors
    int adcLvl      = analogRead(PIN_LEVEL_SENSOR);
    int adcPress    = analogRead(PIN_PRESSURE_SENSOR);
    int adcPH       = analogRead(PIN_PH_SENSOR);
    int adcCL       = analogRead(PIN_CL_SENSOR);
    int adcTurbIn   = analogRead(PIN_TURBIDITY_INTAKE);
    int adcTurbSed  = analogRead(PIN_TURBIDITY_AFTER_SED);
    int adcTurbFilt = analogRead(PIN_TURBIDITY_AFTER_FILT);

    float levelPct       = levelPercentFromADC(adcLvl);
    float pressureBar    = pressureFromADC(adcPress);
    float phValue        = pHFromADC(adcPH);
    float clValue        = chlorineFromADC(adcCL);
    float turbidityIn    = turbidityFromADC(adcTurbIn);
    float turbiditySed   = turbidityFromADC(adcTurbSed);
    float turbidityFilt  = turbidityFromADC(adcTurbFilt);

    // ------------ CONTROL RULES ------------

    // RAW-WATER PUMP (fill storage tank)
    static bool rawPumpOn = false;
    const float LEVEL_START_RAWPUMP = 70.0; // %
    const float LEVEL_STOP_RAWPUMP  = 90.0; // %

    if (!rawPumpOn && levelPct < LEVEL_START_RAWPUMP)
      rawPumpOn = true;
    else if (rawPumpOn && levelPct > LEVEL_STOP_RAWPUMP)
      rawPumpOn = false;

    setRelay(PIN_RELAY_RAW_PUMP, rawPumpOn);

    // BOOSTER PUMP (distribution pressure)
    static bool boosterOn = false;
    const float PRESSURE_LOW    = 2.5; // bar
    const float PRESSURE_HIGH   = 3.0; // bar
    const float LEVEL_MIN_FOR_BOOST = 20.0; // minimum tank level to allow pumping

    if (!boosterOn && (pressureBar < PRESSURE_LOW) && (levelPct > LEVEL_MIN_FOR_BOOST))
      boosterOn = true;
    else if (boosterOn && (pressureBar > PRESSURE_HIGH || levelPct < 10.0))
      boosterOn = false;

    setRelay(PIN_RELAY_BOOSTER_PUMP, boosterOn);

    // pH CORRECTION PUMP
    static bool phPumpOn = false;
    const float PH_LOW  = 6.8;
    const float PH_HIGH = 7.2;  // turn off once above this

    if (!phPumpOn && phValue < PH_LOW)
      phPumpOn = true;
    else if (phPumpOn && phValue > PH_HIGH)
      phPumpOn = false;

    setRelay(PIN_RELAY_PH_PUMP, phPumpOn);

    // CHLORINE DOSING PUMP
    static bool clPumpOn = false;
    const float CL_LOW   = 0.30;  // mg/L
    const float CL_HIGH  = 0.60;  // mg/L

    if (!clPumpOn && clValue < CL_LOW)
      clPumpOn = true;
    else if (clPumpOn && clValue > CL_HIGH)
      clPumpOn = false;

    setRelay(PIN_RELAY_CL_PUMP, clPumpOn);

    // ANTI-CORROSION PUMP: ON if any main pump runs
    bool antiCorrOn = rawPumpOn || boosterOn;
    setRelay(PIN_RELAY_ANTICORR_PUMP, antiCorrOn);

    // ------------ SERIAL DEBUG ------------

    Serial.println("------ STATUS ------");
    Serial.print("Level: ");     Serial.print(levelPct);      Serial.println(" %");
    Serial.print("Pressure: ");  Serial.print(pressureBar);   Serial.println(" bar");
    Serial.print("pH: ");        Serial.println(phValue);
    Serial.print("Cl: ");        Serial.print(clValue);       Serial.println(" mg/L");
    Serial.print("Turbidity In / AfterSed / AfterFilt: ");
    Serial.print(turbidityIn);   Serial.print(" / ");
    Serial.print(turbiditySed);  Serial.print(" / ");
    Serial.println(turbidityFilt);

    Serial.print("Flow Intake / AfterFilt / Dist [L/min]: ");
    Serial.print(flowIntake_Lmin);    Serial.print(" / ");
    Serial.print(flowAfterFilt_Lmin); Serial.print(" / ");
    Serial.println(flowDist_Lmin);

    Serial.print("Raw pump: ");     Serial.println(rawPumpOn ? "ON" : "OFF");
    Serial.print("Booster: ");      Serial.println(boosterOn ? "ON" : "OFF");
    Serial.print("pH pump: ");      Serial.println(phPumpOn ? "ON" : "OFF");
    Serial.print("Cl pump: ");      Serial.println(clPumpOn ? "ON" : "OFF");
    Serial.print("AntiCorr pump: ");Serial.println(antiCorrOn ? "ON" : "OFF");
    Serial.println();
  }
}
