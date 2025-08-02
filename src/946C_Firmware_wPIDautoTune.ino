// Reflow controller firmware with PID control and AutoTune integration
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <SPI.h>
#include <ezButton.h>
#include <math.h>
#include "MAX6675.h"
#include <PID_v1_bc.h>
#include <PID_AutoTune_v0.h>

// Firmware version
const String FW_VERSION = "V5.6.3-PID";

// Pin definitions
#define DSO_TRIG      4
#define RotaryCLK     27
#define RotaryDT      32
#define RotarySW      33
#define SSR_pin       2
#define Fan_pin       26
#define MAX_CS        13
#define MAX_SO        21
#define MAX_CLK       3
#define TFT_ON        15

// TFT and graph settings
TFT_eSPI tft = TFT_eSPI();
const int tftX = 320, tftY = 240;
const int yGraph = tftY - 13, xGraph = 18;
double tempPixelFactor = 250.0 / (tftY - (60 + 13));
double timePixelFactor = 360.0 / (tftX - (18 + 2));

// Colors
#define BLACK   TFT_BLACK
#define WHITE   TFT_WHITE
#define RED     TFT_RED
#define GREEN   TFT_GREEN
#define ORANGE  TFT_ORANGE
#define BLUE    TFT_BLUE
#define YELLOW  TFT_YELLOW
#define DGREEN  0x046B
#define VLGREY  0xDF3D
#define BGGREEN 0xD75C
#define CYAN    0x07FF
#define DGREY   TFT_DARKGREY
#define OFF     0
#define ON      255

// corner radius for all round‐rectangle UI elements
const uint8_t RectRadius = 5;   // pick whatever radius you like
boolean redrawCurve = false;

// Thermocouple
MAX6675 thermoCouple(MAX_CS, MAX_SO, MAX_CLK);
// Declare the variable to store the Celsius temperature
float TCCelsius = 0.0;

// Rotary encoder
ezButton button(RotarySW);
int CLKNow, CLKPrevious, DTNow, DTPrevious;
volatile int itemCounter = -1, previousItemCounter = 0;
bool menuChanged = false, editMode = false;

// Selection of the fields by the rotation of the encoder
bool preheatTempSelected        = false;
bool preheatTimeSelected        = false;
bool soakingTempSelected        = false;
bool soakingTimeSelected        = false;
bool reflowTempSelected         = false;
bool reflowTimeSelected         = false;
bool coolingTempSelected        = false;
bool coolingTimeSelected        = false;
bool warmupTempSelected         = false;
bool freeWarmUpButtonSelected   = false;
bool startStopButtonSelected    = false;
bool freeHeatingTargetSelected  = false;
bool freeHeatingOnOffSelected   = false;
bool freeCoolingTargetSelected  = false;
bool freeCoolingOnOffSelected   = false;
bool solderpasteFieldSelected   = false;

// Solder paste profiles
struct solderpaste {
  char pasteName[30];
  int preheatTemp, preheatTime;
  int soakingTemp, soakingTime;
  int reflowTemp, reflowTime;
  int coolingTemp, coolingTime;
};
solderpaste solderpastes[] = {
  { "Sn42/Bi57.6/Ag0.4",
   90,90,
   130,180,
   165,240,
   165,250 
   },
  { "Sn42/Bi57/Ag1",    
   90,90,
  130,180,
  165,240,
  165,250 
  },
  { "Sn63/Pb37",        
  100,30,
  150,120,
  235,210,
  235,220 
  },
  { "Sn63/Pb37 Mod",    
  100,60,
  150,120,
  235,210,
  235,220 
  },
  { "Sn42/Bi58/CW",     
   80,60,
   110,180,
   165,240,
   165,250 
   }
};
int solderPasteSelected = 0, prev_solderPasteSelected = 0;
int numSolderpastes;

// Active curve values
volatile int preheatTemp, preheatTime;
volatile int soakingTemp, soakingTime;
volatile int reflowTemp, reflowTime;
volatile int coolingTemp, coolingTime;

// Pixel positions
int preheatTemp_px, preheatTime_px;
int soakingTemp_px, soakingTime_px;
int reflowTemp_px, reflowTime_px;
int coolingTemp_px, coolingTime_px;
int measuredTemp_px, measuredTime_px;

// Control flags and timing
bool coolingFanEnabled = false, heatingEnabled = false;
bool reflow = false, enableFreeHeating = false;
bool enableFreeCooling = false, enableWarmup = false;
unsigned long temperatureTimer = 0;
unsigned long SSRTimer = 0;
const unsigned long SSRInterval = 250;
double elapsedHeatingTime = 0;

// Phases
enum ReflowPhase { PREHEAT = 0, SOAK, REFLOW, HOLD, COOLING };
ReflowPhase currentPhase = PREHEAT;

// Forward-cutoff times
int preheatCutOff, reflowCutOff;
const int preheatCutOffTime = 15, reflowCutOffTime = 15;

// Free modes default targets
volatile int freeHeatingTemp = 200;
volatile int freeCoolingTemp = 40;
volatile int warmupTemp      = 38;

// PID and AutoTune
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input, &Output);
bool usePID = true;
bool autoTuneRunning = false;
unsigned long autoTuneStartTime = 0;

// Function prototypes (abbreviated)
void setup();
void loop();
void rotaryEncoderISR();
void processRotaryButton();
void updateHighlighting();
void runReflow();
void freeHeating();
void freeCooling();
void runWarmup();
void drawAxis();
void drawCurve();
void drawFreeCurve();
void drawReflowCurve();
void drawActionButtons();
void measureTemperature();
void removeFieldsFromDisplay();
void updateStatus(int, int, const char*);
void printTemp();
void printTargetTemperature();
void printElapsedTime();
void printPWM();
void printFan();

//==============================================================================
void setup() {
  pinMode(TFT_ON, OUTPUT);  digitalWrite(TFT_ON, LOW);
  Serial.begin(9600); while (!Serial);
  delay(5000);
  Serial.println("\n\nReflow controller " + FW_VERSION);
  SPI.begin();
  pinMode(DSO_TRIG, OUTPUT);
  pinMode(RotaryCLK, INPUT);
  pinMode(RotaryDT, INPUT);
  button.setDebounceTime(20);
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), rotaryEncoderISR, CHANGE);
  CLKPrevious = digitalRead(RotaryCLK);
  DTPrevious = digitalRead(RotaryDT);
  pinMode(SSR_pin, OUTPUT); analogWrite(SSR_pin, OFF);
  pinMode(Fan_pin, OUTPUT); digitalWrite(Fan_pin, HIGH);
  Serial.println("Init TFT");
  digitalWrite(TFT_ON, HIGH);
  tft.init(); tft.setRotation(3); tft.fillScreen(BLACK);
  thermoCouple.begin(); thermoCouple.setSPIspeed(40000000);

  // Load solder pastes
  numSolderpastes = sizeof(solderpastes) / sizeof(solderpaste);
  solderpaste cur = solderpastes[solderPasteSelected];
  preheatTemp = cur.preheatTemp;  preheatTime  = cur.preheatTime;
  soakingTemp = cur.soakingTemp;  soakingTime  = cur.soakingTime;
  reflowTemp  = cur.reflowTemp;   reflowTime   = cur.reflowTime;
  coolingTemp = cur.coolingTemp;  coolingTime  = cur.coolingTime;

  preheatCutOff = preheatTime - preheatCutOffTime;
  reflowCutOff  = reflowTime  - reflowCutOffTime;

  // PID setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  // Welcome screen
  tft.setTextColor(WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Automated reflow " + FW_VERSION, tft.width()/2, 40, 2);
  tft.drawString("2025 paulv", tft.width()/2, 80, 2);
  tft.drawString("curiousscientist.tech", tft.width()/2, 120, 2);
  tft.setTextDatum(TL_DATUM);
  vTaskDelay(3000/portTICK_PERIOD_MS);

  // Draw initial curve and controls
  tft.fillScreen(BLACK);
  drawReflowCurve();
  drawActionButtons();
  digitalWrite(Fan_pin, LOW);
  Serial.println("Setup done");
}

//==============================================================================
void loop() {
  button.loop();
  measureTemperature();
  updateHighlighting();
  runReflow();
  runWarmup();
  freeHeating();
  freeCooling();
  if (button.isPressed()) processRotaryButton();

  // AutoTune runtime
  if (autoTuneRunning) {
    if (aTune.Runtime()) {
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp, Ki, Kd);
      autoTuneRunning = false;
      updateStatus(GREEN, BLACK, "AutoTune DONE");
    }
  }
}

//==============================================================================
void rotaryEncoderISR() {
  CLKNow = digitalRead(RotaryCLK);

  // ... [existing field‐editing logic unchanged] ...

  // Navigation when no field selected
  if (!preheatTempSelected && !preheatTimeSelected && !soakingTempSelected
   && !soakingTimeSelected && !reflowTempSelected   && !reflowTimeSelected
   && !coolingTempSelected && !coolingTimeSelected  && !warmupTempSelected
   && !freeWarmUpButtonSelected                     && !freeHeatingTargetSelected
   && !startStopButtonSelected                      && !freeHeatingOnOffSelected
   && !freeCoolingTargetSelected                    && !freeCoolingOnOffSelected
   && !solderpasteFieldSelected) {
    if (CLKNow != CLKPrevious && CLKNow == 1) {
      previousItemCounter = itemCounter;
      if (digitalRead(RotaryDT) != CLKNow) {
        if (itemCounter > 0) itemCounter--;
        else                 itemCounter = 16;
      } else {
        if (itemCounter < 16) itemCounter++;
        else                  itemCounter = 0;
      }
    }
    menuChanged = true;
  }
  CLKPrevious = CLKNow;
}

//==============================================================================
void processRotaryButton() {
  switch (itemCounter) {
    // … [cases 0–15 unchanged] …

    case 16:  // AutoTune trigger
      if (!autoTuneRunning) {
        autoTuneRunning = true;
        aTune.SetOutputStep(50);
        aTune.SetControlType(1);  // PID
        aTune.SetLookbackSec(10);
        aTune.SetNoiseBand(1);
        updateStatus(ORANGE, BLACK, "AutoTune ON");
      } else {
        aTune.Cancel();
        autoTuneRunning = false;
        updateStatus(RED, WHITE, "AutoTune OFF");
      }
      break;
  }
  menuChanged = false;
}

//==============================================================================
void updateHighlighting() {
  if (!menuChanged) return;
  // Highlight new field
  switch (itemCounter) {
    // … [cases 0–15 unchanged] …
    case 16:
      tft.fillRoundRect(220, 80, 100, 15, RectRadius, YELLOW);
      tft.setTextColor(BLACK);
      tft.drawString("AutoTune", 225, 82, 2);
      break;
  }
  // Remove highlight from previous field
  switch (previousItemCounter) {
    // … [cases 0–15 unchanged] …
    case 16:
      tft.fillRoundRect(220, 80, 100, 15, RectRadius, BLACK);
      tft.setTextColor(WHITE);
      tft.drawString("AutoTune", 225, 82, 2);
      break;
  }
  menuChanged = false;
}

//==============================================================================
void runReflow() {
  if (!reflow) return;
  if (elapsedHeatingTime >= 340) return;

  unsigned long now = millis();
  if (now - SSRTimer < SSRInterval) return;

  // Plot measurement
  measuredTemp_px = yGraph - (int)(TCCelsius / tempPixelFactor);
  measuredTime_px = xGraph + (int)(elapsedHeatingTime / timePixelFactor);
  tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
  printElapsedTime();

  // Determine targetTemp
  switch (currentPhase) {
    case PREHEAT:
      Setpoint = 20 + (elapsedHeatingTime / preheatTime) * (preheatTemp - 20);
      updateStatus(DGREEN, WHITE, "Preheat");
      break;
    case SOAK:
      Setpoint = preheatTemp + ((elapsedHeatingTime - preheatTime) / (soakingTime - preheatTime)) * (soakingTemp - preheatTemp);
      updateStatus(DGREEN, WHITE, "Soaking");
      break;
    case REFLOW:
      Setpoint = soakingTemp + ((elapsedHeatingTime - soakingTime) / (reflowTime - soakingTime)) * (reflowTemp - soakingTemp);
      updateStatus(DGREEN, WHITE, "Reflow");
      break;
    case HOLD:
      Setpoint = reflowTemp + ((elapsedHeatingTime - reflowTime) / (coolingTime - reflowTime)) * (coolingTemp - reflowTemp);
      updateStatus(DGREEN, WHITE, "Holding");
      break;
    case COOLING:
      Setpoint = 40;
      updateStatus(DGREEN, WHITE, "Cooling");
      break;
  }

  // Control output
  if (usePID) {
    Input = TCCelsius;
    myPID.Compute();
    analogWrite(SSR_pin, int(Output));
  } else {
    analogWrite(SSR_pin, (TCCelsius >= Setpoint) ? 0 : 255);
  }

  // Advance phase
  if (currentPhase == PREHEAT && TCCelsius > preheatTemp && elapsedHeatingTime > preheatTime)
    currentPhase = SOAK;
  if (currentPhase == SOAK   && TCCelsius > soakingTemp && elapsedHeatingTime > soakingTime)
    currentPhase = REFLOW;
  if (currentPhase == REFLOW && (TCCelsius > reflowTemp || elapsedHeatingTime > reflowTime))
    currentPhase = HOLD;
  if (currentPhase == HOLD   && (TCCelsius > coolingTemp && elapsedHeatingTime > coolingTime))
    currentPhase = COOLING;

  // Fan control in COOLING
  if (currentPhase == COOLING) {
    if (TCCelsius > Setpoint) {
      digitalWrite(Fan_pin, HIGH);
      printFan();
    } else {
      digitalWrite(Fan_pin, LOW);
      printFan();
    }
  } else {
    printPWM();
  }

  elapsedHeatingTime += SSRInterval / 1000.0;
  SSRTimer = now;
}

//==============================================================================
void freeHeating() {
  if (!enableFreeHeating) return;
  unsigned long now = millis();
  if (now - SSRTimer < SSRInterval) return;

  measuredTemp_px = yGraph - (int)(TCCelsius / tempPixelFactor);
  measuredTime_px = xGraph + (int)(elapsedHeatingTime / timePixelFactor);
  tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
  tft.drawPixel(measuredTime_px, measuredTemp_px+1, CYAN);
  printElapsedTime();
  Setpoint = freeHeatingTemp;
  printTargetTemperature();
  updateStatus(DGREEN, WHITE, "Heating");

  if (usePID) {
    Input = TCCelsius;
    myPID.Compute();
    analogWrite(SSR_pin, int(Output));
  } else {
    analogWrite(SSR_pin, (TCCelsius >= Setpoint) ? 0 : 255);
  }

  printPWM();
  elapsedHeatingTime += SSRInterval / 1000.0;
  SSRTimer = now;
}

//==============================================================================
void freeCooling() {
  if (!enableFreeCooling) return;
  unsigned long now = millis();
  if (now - SSRTimer < SSRInterval) return;

  measuredTemp_px = yGraph - (int)(TCCelsius / tempPixelFactor);
  measuredTime_px = xGraph + (int)(elapsedHeatingTime / timePixelFactor);
  tft.drawPixel(measuredTime_px, measuredTemp_px, BLUE);
  tft.drawPixel(measuredTime_px, measuredTemp_px+1, BLUE);
  printElapsedTime();
  Setpoint = freeCoolingTemp;
  printTargetTemperature();
  updateStatus(DGREEN, WHITE, "Cooling");

  if (TCCelsius > Setpoint) digitalWrite(Fan_pin, HIGH);
  else                  digitalWrite(Fan_pin, LOW);

  printFan();
  elapsedHeatingTime += SSRInterval / 1000.0;
  SSRTimer = now;
}

//==============================================================================
void runWarmup() {
  if (!enableWarmup) return;
  unsigned long now = millis();
  if (now - SSRTimer < SSRInterval) return;

  measuredTemp_px = yGraph - (int)(TCCelsius / tempPixelFactor);
  measuredTime_px = xGraph + (int)(elapsedHeatingTime / timePixelFactor);
  tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
  tft.drawPixel(measuredTime_px, measuredTemp_px+1, CYAN);
  printElapsedTime();
  Setpoint = warmupTemp;
  printTargetTemperature();
  updateStatus(DGREEN, WHITE, "Warmup");

  if (usePID) {
    Input = TCCelsius;
    myPID.Compute();
    analogWrite(SSR_pin, int(Output));
  } else {
    analogWrite(SSR_pin, (TCCelsius >= Setpoint) ? 0 : 125);
  }
  printPWM();
  elapsedHeatingTime += SSRInterval / 1000.0;
  SSRTimer = now;
}

//==============================================================================
void drawAxis() {
  tft.drawLine(xGraph, (int)238 - (250/tempPixelFactor)-12, xGraph,238-13, RED);
  for (int y=50; y<=250; y+=50) {
    int yp = (int)yGraph - (y/tempPixelFactor);
    tft.drawLine(13, yp, 22, yp, RED);
    tft.drawString(String(y), 0, yp-10, 1);
  }
  tft.drawLine(xGraph, yGraph, xGraph + (int)(360/timePixelFactor), yGraph, WHITE);
  for (int s=30; s<=330; s+=30) {
    int xp = xGraph + (int)(s/timePixelFactor);
    int h = (s%60==0 ? 220 : 222);
    tft.drawLine(xp, 226, xp, h, WHITE);
    if (s%60==0) tft.drawString(String(s), xp-5,230,1);
  }
  tft.drawString("`C",4,tftY-35,2);
  tft.drawString("seconds",15,tftY-10,1);
}

//==============================================================================
void drawCurve() {
  drawAxis();
  tft.drawLine(xGraph, yGraph - (20/tempPixelFactor), preheatTime_px, preheatTemp_px, YELLOW);
  tft.drawLine(preheatTime_px, preheatTemp_px, soakingTime_px, soakingTemp_px, ORANGE);
  tft.drawLine(soakingTime_px, soakingTemp_px, reflowTime_px, reflowTemp_px, RED);
  tft.drawLine(reflowTime_px, reflowTemp_px, coolingTime_px, coolingTemp_px, RED);
  tft.drawLine(coolingTime_px, coolingTemp_px, coolingTime_px+40, coolingTemp_px+20, BLUE);
}

//==============================================================================
void drawFreeCurve() {
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.drawString(solderpastes[solderPasteSelected].pasteName,50,1,2);
  drawAxis();
}

//==============================================================================
void drawReflowCurve() {
  if (!redrawCurve) return;
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.drawString(solderpastes[solderPasteSelected].pasteName,50,1,2);

  preheatTemp_px = yGraph - (int)(preheatTemp/tempPixelFactor);
  preheatTime_px = xGraph + (int)(preheatTime/timePixelFactor);
  soakingTemp_px = yGraph - (int)(soakingTemp/tempPixelFactor);
  soakingTime_px = xGraph + (int)(soakingTime/timePixelFactor);
  reflowTemp_px  = yGraph - (int)(reflowTemp/timePixelFactor);
  reflowTime_px  = xGraph + (int)(reflowTime/timePixelFactor);
  coolingTemp_px = yGraph - (int)(coolingTemp/timePixelFactor);
  coolingTime_px = xGraph + (int)(coolingTime/timePixelFactor);

  drawCurve();

  // labels
  tft.setTextColor(RED);
  tft.setCursor(preheatTime_px-10,preheatTemp_px-30);
  tft.printf("%dC",preheatTemp);
  tft.setTextColor(WHITE);
  tft.setCursor(preheatTime_px-10,preheatTemp_px-20);
  tft.printf("%ds",preheatTime);

  tft.setTextColor(RED);
  tft.setCursor(soakingTime_px-25,soakingTemp_px-20);
  tft.printf("%dC",soakingTemp);
  tft.setTextColor(WHITE);
  tft.setCursor(soakingTime_px-25,soakingTemp_px-10);
  tft.printf("%ds",soakingTime);

  tft.setTextColor(RED);
  tft.setCursor(reflowTime_px-5,reflowTemp_px+10);
  tft.printf("%dC",reflowTemp);
  tft.setTextColor(WHITE);
  tft.setCursor(reflowTime_px-5,reflowTemp_px+20);
  tft.printf("%ds",reflowTime);

  tft.setTextColor(BLUE);
  tft.setCursor(coolingTime_px+20,coolingTemp_px+30);
  tft.printf("%dC",coolingTemp);
  tft.setTextColor(WHITE);
  tft.setCursor(coolingTime_px+20,coolingTemp_px+20);
  tft.printf("%ds",coolingTime);

  drawActionButtons();
  redrawCurve = false;
}

//==============================================================================
void drawActionButtons() {
  // Warmup
  tft.fillRoundRect(260,0,60,15,2,DGREEN);
  tft.setTextColor(WHITE); tft.drawString("WARMUP",265,0,2);
  tft.fillRoundRect(220,2,32,12,2,BLACK);
  tft.setTextColor(RED);   tft.printf("%dC",warmupTemp); tft.drawNumber(warmupTemp,228,4,1);

  // Reflow
  tft.fillRoundRect(260,20,60,15,2,ORANGE);
  tft.setTextColor(WHITE); tft.drawString("REFLOW",265,20,2);

  // Heating
  tft.fillRoundRect(260,40,60,15,2,RED);
  tft.setTextColor(WHITE); tft.drawString("HEATING",265,40,2);
  tft.fillRoundRect(220,42,32,12,2,BLACK);
  tft.setTextColor(RED);   tft.printf("%dC",freeHeatingTemp);

  // Cooling
  tft.fillRoundRect(260,60,60,15,2,BLUE);
  tft.setTextColor(WHITE); tft.drawString("COOLING",265,60,2);
  tft.fillRoundRect(220,62,32,12,2,BLACK);
  tft.setTextColor(BLUE);  tft.printf("%dC",freeCoolingTemp);

  // AutoTune
  tft.fillRoundRect(220,80,100,15,2,ORANGE);
  tft.setTextColor(BLACK); tft.drawString("AutoTune",225,82,2);
}

//==============================================================================
void measureTemperature() {
  if (millis() - temperatureTimer < 250) return;
  int status = thermoCouple.read();
  if (status) Serial.printf("Max status: %d\n",status);
  TCCelsius = thermoCouple.getTemperature();
  Serial.printf("Temp: %.1f\n",TCCelsius);
  printTemp();
  temperatureTimer = millis();
}

//==============================================================================
void removeFieldsFromDisplay() {
  tft.fillRoundRect(preheatTime_px-10,preheatTemp_px-30,24,9,2,BLACK);
  tft.fillRoundRect(preheatTime_px-10,preheatTemp_px-20,24,9,2,BLACK);
  tft.fillRoundRect(soakingTime_px-25,soakingTemp_px-20,24,9,2,BLACK);
  tft.fillRoundRect(soakingTime_px-25,soakingTemp_px-10,24,9,2,BLACK);
  tft.fillRoundRect(reflowTime_px-5,reflowTemp_px+10,24,9,2,BLACK);
  tft.fillRoundRect(reflowTime_px-5,reflowTemp_px+20,24,9,2,BLACK);
  tft.fillRoundRect(coolingTime_px+20,coolingTemp_px+30,24,9,2,BLACK);
  tft.fillRoundRect(coolingTime_px+20,coolingTemp_px+20,24,9,2,BLACK);
  tft.fillRoundRect(220,0,100,15,2,BLACK);
  tft.fillRoundRect(220,40,100,15,2,BLACK);
  tft.fillRoundRect(220,60,100,15,2,BLACK);
}

//==============================================================================
void updateStatus(int fieldColor, int textColor, const char* text) {
  tft.fillRoundRect(160,190,70,18,2,fieldColor);
  tft.setTextColor(textColor);
  tft.drawString(text,170,190,2);
}

//==============================================================================
void printTemp() {
  tft.fillRoundRect(30,40,90,16,2,(TCCelsius>500?RED:DGREEN));
  tft.setTextColor(WHITE);
  if (TCCelsius>500) tft.printf("Err %d",int(TCCelsius));
  else               tft.printf("Temp %d`C",int(TCCelsius));
}

//==============================================================================
void printTargetTemperature() {
  tft.fillRoundRect(30,60,80,16,2,DGREEN);
  tft.setTextColor(WHITE);
  tft.printf("Targt %d`C",int(Setpoint));
}

//==============================================================================
void printElapsedTime() {
  tft.fillRoundRect(120,40,80,16,2,DGREEN);
  tft.setTextColor(WHITE);
  tft.printf("Time %ds",int(elapsedHeatingTime));
}

//==============================================================================
void printPWM() {
  tft.fillRoundRect(120,60,80,16,2,DGREEN);
  tft.setTextColor(WHITE);
  tft.printf("PID : %d",int(Output));
}

//==============================================================================
void printFan() {
  tft.fillRoundRect(120,60,80,16,2,DGREEN);
  tft.setTextColor(WHITE);
  tft.printf("FAN : %s", coolingFanEnabled?"ON":"OFF");
}
// ============== End of firmware ==============
