#include <Wire.h>
#include "MAX30105.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

MAX30105 sensor;
TFT_eSPI tft = TFT_eSPI();

// ===== SYSTEM MODES =====
enum DeviceMode { MODE_PULSE_OX, MODE_ECG };
DeviceMode currentMode = MODE_PULSE_OX;
const int BUTTON_PIN = 12;
bool lastBtnState = HIGH;

// ===== AD8232 ECG CONFIG =====
const int ECG_ANALOG_PIN = 36; // VP Pin
const int LO_MINUS_PIN = 14;
const int LO_PLUS_PIN = 27;
float ecgFiltered = 185;
float ecgPrev = 185;

// ===== MAX30105 OXIMETER VARIABLES =====
float irValue, redValue;
float filteredIR = 0, prevFilteredIR = 0, dcIR = 0;
float filteredRed = 0, prevFilteredRed = 0, dcRed = 0;
float irMax = -99999, irMin = 99999;
float redMax = -99999, redMin = 99999;

// ===== BEAT DETECTION =====
unsigned long lastBeatTime = 0;
float bpmSmooth = 75;
float threshold = 1500;
bool rising = false;
bool beatDetected = false;
float spo2Smooth = 98.0;

// ===== DISPLAY BOUNDARIES =====
int x = 0;
int lastY = 220;
const int graphTop = 130;
const int graphBottom = 310;
const int graphHeight = 180;

// ===== STATE TRACKING =====
bool fingerPresent = false;
unsigned long lastSignalTime = 0;
bool modeChanged = false;

// Subtle coordinate dot background matrix generator
void drawGridSlice(int currentX) {
  for (int gY = graphTop; gY <= graphBottom; gY += 20) {
    if (gY == 210) { 
      tft.drawPixel(currentX, gY, TFT_NAVY); // Center baseline
    } else {
      tft.drawPixel(currentX, gY, tft.color565(40, 40, 40)); 
    }
  }
}

// Redraw layout headers from scratch on mode change
void drawStaticUI() {
  tft.fillScreen(TFT_BLACK);
  x = 0; // Reset trace sweep window

  // Top Title Banner
  tft.fillRect(0, 0, 240, 25, tft.color565(30, 30, 30));
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 8);
  tft.print("VITAL SIGNS MONITOR");

  tft.setTextSize(2); // Keeping layout at size 2 for ease of viewing
  if (currentMode == MODE_PULSE_OX) {
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(15, 45);  tft.print("PR (BPM)");
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(135, 45); tft.print("SpO2 (%)");
    
    // Initial dashes
    tft.setTextSize(4);
    tft.setTextColor(tft.color565(60, 60, 20), TFT_BLACK);  tft.setCursor(15, 65);  tft.print("---");
    tft.setTextColor(tft.color565(20, 60, 60), TFT_BLACK); tft.setCursor(135, 65); tft.print("---");
    lastY = 220;
  } 
  else if (currentMode == MODE_ECG) {
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(15, 45); tft.print("ECG TRACE MODE");
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(15, 65); tft.setTextSize(3); tft.print("ACTIVE LEAD");
    lastY = 220;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  // Setup UI Control Switches
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);

  tft.init();
  tft.setRotation(0);
  
  drawStaticUI();

  Wire.begin();
  if (sensor.begin(Wire)) {
    sensor.setup(0x14, 1, 2, 50, 411, 4096); // Optimized steady-stream config
  } else {
    tft.setCursor(10, 200); tft.setTextSize(2); tft.print("OX Sensor Missing");
  }
}

void loop() {
  // ----- 1. INTERRUPT CHECK: CYCLE UI BUTTON -----
  bool btnState = digitalRead(BUTTON_PIN);
  if (btnState == LOW && lastBtnState == HIGH) {
    delay(50); // Simple debounce
    currentMode = (currentMode == MODE_PULSE_OX) ? MODE_ECG : MODE_PULSE_OX;
    drawStaticUI();
  }
  lastBtnState = btnState;

  // ----- 2. DATA PROCESSING BRANCHES -----
  int y = 220; // Default graph line positioning value

  if (currentMode == MODE_PULSE_OX) {
    // === PULSE OXIMETER MODULE SUBROUTINE ===
    if (sensor.available()) {
      irValue = sensor.getIR();
      redValue = sensor.getRed();
      sensor.nextSample();

      if (irValue > 30000) {
        fingerPresent = true;
        lastSignalTime = millis();
      } else {
        fingerPresent = false;
      }

      // Math Filters
      dcIR = dcIR * 0.95 + irValue * 0.05;
      filteredIR = irValue - dcIR;
      filteredIR = (filteredIR + prevFilteredIR) * 0.5;
      prevFilteredIR = filteredIR;

      dcRed = dcRed * 0.95 + redValue * 0.05;
      filteredRed = redValue - dcRed;
      filteredRed = (filteredRed + prevFilteredRed) * 0.5;
      prevFilteredRed = filteredRed;

      if (fingerPresent) {
        if (filteredIR > irMax) irMax = filteredIR;
        if (filteredIR < irMin) irMin = filteredIR;
        if (filteredRed > redMax) redMax = filteredRed;
        if (filteredRed < redMin) redMin = filteredRed;

        // Peak Analysis
        if (filteredIR > threshold && !rising) {
          rising = true;
          unsigned long now = millis();
          unsigned long interval = now - lastBeatTime;
          if (interval > 300 && interval < 2000) { 
            bpmSmooth = bpmSmooth * 0.7 + (60000.0 / interval) * 0.3;
            beatDetected = true;

            float irAC = irMax - irMin;
            float redAC = redMax - redMin;
            if (irAC > 100 && redAC > 100 && dcIR > 0 && dcRed > 0) {
              float ratio = (redAC / dcRed) / (irAC / dcIR);
              float spo2Calc = 110.0 - (25.0 * ratio); 
              if (spo2Calc > 100.0) spo2Calc = 100.0;
              if (spo2Calc < 70.0) spo2Calc = 70.0;
              spo2Smooth = spo2Smooth * 0.8 + spo2Calc * 0.2;
            }
          }
          lastBeatTime = now;
          irMax = -99999; irMin = 99999; redMax = -99999; redMin = 99999;
        }
      } else {
        rising = false;
        bpmSmooth = 75;
        spo2Smooth = 98.0;
      }

      if (filteredIR < threshold) rising = false;
      threshold = threshold * 0.92 + abs(filteredIR) * 0.08;

      // Update Screen Text
      if (fingerPresent) {
        tft.setTextSize(4);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(15, 65);   tft.print((int)bpmSmooth); tft.print("  ");
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setCursor(135, 65);  tft.print((int)spo2Smooth); tft.print("% ");
      }

      if (millis() - lastSignalTime > 2000) {
        tft.setTextSize(4);
        tft.setTextColor(tft.color565(60, 60, 20), TFT_BLACK);  tft.setCursor(15, 65);  tft.print("---  ");
        tft.setTextColor(tft.color565(20, 60, 60), TFT_BLACK); tft.setCursor(135, 65); tft.print("--% ");
      }

      // Frame Wave Calculations
      if (fingerPresent) {
        float displayWave = filteredIR * 0.008; 
        y = 220 - (int)(displayWave * 20); 
        if (beatDetected) y -= 10;
      } else {
        y = 220;
        if (lastY != 220) lastY = 220;
      }
    }
  } 
  else if (currentMode == MODE_ECG) {
    // === AD8232 ECG MODULE SUBROUTINE ===
    bool leadsOff = (digitalRead(LO_PLUS_PIN) == 1 || digitalRead(LO_MINUS_PIN) == 1);

    if (leadsOff) {
      // Clear data array and warn user
      tft.setCursor(15, 65); tft.setTextSize(3);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print("LEADS DISCONN");
      y = 220; // Flatline
      if (lastY != 220) lastY = 220;
    } else {
      // Clear alert text cleanly when leads connect
      tft.setCursor(15, 65); tft.setTextSize(3);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.print("SIGNAL ACTIVE");

      int rawAnalog = analogRead(ECG_ANALOG_PIN); // Reads raw 0-4095 scale
      
      // Pass raw voltage through a quick low pass smoothing filter
      ecgFiltered = ecgFiltered * 0.85 + rawAnalog * 0.15;
      
      // Map the 12-bit analog input window cleanly into our 180px height screen grid bounds
      // High values travel down on TFTs, so subtract from base reference
      y = 220 - (int)((ecgFiltered - 2000) * 0.07); 
    }
  }

  // ----- 3. GENERAL GRAPHING PIPELINE (SHARED BY BOTH MODES) -----
  y = constrain(y, graphTop, graphBottom - 1);

  // Rolling Clear Slice
  tft.fillRect(x + 1, graphTop, 12, graphHeight, TFT_BLACK);
  for (int i = 1; i <= 12; i++) {
    drawGridSlice(x + i);
  }

  // Draw Line Segment matching the current mode color signature
  if (x < 239) {
    uint16_t traceColor = (currentMode == MODE_PULSE_OX) ? TFT_CYAN : TFT_GREEN;
    tft.drawLine(x, lastY, x + 1, y, traceColor);
  }

  lastY = y;
  x++;
  if (x >= 239) x = 0;

  delay(6); // Regulates 1-to-1 sync tracing rate
}