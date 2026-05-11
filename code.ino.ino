#define BLYNK_TEMPLATE_ID "TMPL6XALHHK9S"
#define BLYNK_TEMPLATE_NAME "Power Monitor"
#define BLYNK_AUTH_TOKEN "-vnw2YRgAyK_62T30MVtR6Up9fL2KO-Q"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <PZEM004Tv30.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

char ssid[] = "Ali abotiheen";
char pass[] = "abotiheen";

PZEM004Tv30 pzemGrid(Serial2, 16, 17);
PZEM004Tv30 pzemInv(Serial1, 4, 5);
Adafruit_ADS1115 ads;

LiquidCrystal_I2C lcdAC(0x27, 16, 2); 
LiquidCrystal_I2C lcdDC(0x26, 16, 2); 

#define WATANI_RELAY_PIN 26    
#define INVERTER_RELAY_PIN 27  
#define INV_INPUT_CONTACTOR_PIN 25  
#define ESSENTIAL_LOAD_PIN 33       
#define NON_ESSENTIAL_LOAD_PIN 32   

#define PV_VOLT_CH       0
#define BAT_VOLT_CH      1
#define BAT_CURRENT_CH   2
#define PV_CURRENT_CH    3

#define PV_VOLT_FACTOR   22.3
#define BAT_VOLT_FACTOR  11.0

#define ACS_SENS 0.066
#define CURRENT_ZERO_LIMIT 2.0

float v1, i1, p1, f1, pf1;
float v2, i2, p2, f2, pf2;
float batV, pvV, batI, pvI;
float batPowerW, pvPowerW;
int batSOC;

float batCurrentOffset = 2.4;
float pvCurrentOffset  = 2.4;

bool manualMode = false, isWataniActive = false;
int v14_state = 1, v15_state = 1; 

unsigned long prevMillis = 0;
unsigned long lastWifiTry = 0;

int gridStableCounter = 0;
bool isSwitching = false;
String loadStatus = "FULL";

bool halfLoadNotified = false;
bool battery20Notified = false;
bool allOffNotified = false;
bool batteryFullNotified = false;
bool gridOnNotified = false;
bool gridOffNotified = false;

float readADSVoltageAverage(byte channel) {
  float sum = 0;
  for (int k = 0; k < 20; k++) {
    sum += ads.computeVolts(ads.readADC_SingleEnded(channel));
    delay(2);
  }
  return sum / 20.0;
}

float calibrateCurrentOffset(byte channel) {
  float sum = 0;
  for (int k = 0; k < 300; k++) {
    sum += ads.computeVolts(ads.readADC_SingleEnded(channel));
    delay(5);
  }
  return sum / 300.0;
}

int calculateSoC(float voltage) {
  if (voltage < 10.5) return 0;

  int soc = map((int)(voltage * 10), 110, 127, 0, 100);

  if (soc > 100) soc = 100;
  if (soc < 0) soc = 0;

  return soc;
}

void sendEvent(String eventCode, String message) {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.logEvent(eventCode, message);
  }
}

void connectToBlynkNonBlocking() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    return;
  }

  if (!Blynk.connected()) {
    Blynk.connect(1000);
  }
}

void safeSwitchToGrid() {
  isSwitching = true;
  gridStableCounter = 0; 

  digitalWrite(INVERTER_RELAY_PIN, HIGH); 
  digitalWrite(WATANI_RELAY_PIN, HIGH);

  delay(200); 
  lcdAC.clear();

  for (int j = 7; j > 0; j--) {
    if (Blynk.connected()) Blynk.run();

    lcdAC.setCursor(0, 0); 
    lcdAC.print("To Grid System");

    lcdAC.setCursor(0, 1); 
    lcdAC.print("After: ");
    lcdAC.print(j);
    lcdAC.print(" sec... ");

    delay(1000);
  }

  digitalWrite(WATANI_RELAY_PIN, LOW);
  isWataniActive = true;

  if (!gridOnNotified) {
    sendEvent("grid_available", "Grid power restored. System switched to grid.");
    gridOnNotified = true;
    gridOffNotified = false;
  }

  isSwitching = false;
  lcdAC.init(); 
  lcdAC.backlight();
}

void safeSwitchToInv() {
  isSwitching = true;

  digitalWrite(WATANI_RELAY_PIN, HIGH);
  digitalWrite(INVERTER_RELAY_PIN, HIGH);

  delay(200); 
  lcdAC.clear();

  for (int j = 3; j > 0; j--) {
    if (Blynk.connected()) Blynk.run();

    lcdAC.setCursor(0, 0); 
    lcdAC.print("To Inv System");

    lcdAC.setCursor(0, 1); 
    lcdAC.print("Wait: ");
    lcdAC.print(j);
    lcdAC.print(" sec... ");

    delay(1000);
  }

  digitalWrite(INVERTER_RELAY_PIN, LOW);
  isWataniActive = false;

  if (!gridOffNotified) {
    sendEvent("grid_lost", "Grid power lost. System switched to inverter.");
    gridOffNotified = true;
    gridOnNotified = false;
  }

  isSwitching = false;
  lcdAC.init(); 
  lcdAC.backlight();
}

BLYNK_WRITE(V5) { 
  manualMode = param.asInt(); 
}

BLYNK_WRITE(V6) { 
  if (manualMode && !isSwitching && param.asInt() == 1) {
    safeSwitchToGrid(); 
  }
} 

BLYNK_WRITE(V7) { 
  if (manualMode && !isSwitching && param.asInt() == 1) {
    safeSwitchToInv(); 
  }
}

BLYNK_WRITE(V13) { 
  digitalWrite(INV_INPUT_CONTACTOR_PIN, param.asInt() ? LOW : HIGH); 
}

BLYNK_WRITE(V14) { 
  v14_state = param.asInt(); 
} 

BLYNK_WRITE(V15) { 
  v15_state = param.asInt(); 
}

void setup() {
  Wire.begin(13, 14);

  lcdAC.init(); 
  lcdAC.backlight();

  lcdDC.init(); 
  lcdDC.backlight();

  Wire1.begin(21, 22);
  ads.begin(0x48, &Wire1);

  lcdDC.clear();
  lcdDC.setCursor(0, 0);
  lcdDC.print("Calibrating DC");
  lcdDC.setCursor(0, 1);
  lcdDC.print("No Load Please");

  batCurrentOffset = calibrateCurrentOffset(BAT_CURRENT_CH);
  pvCurrentOffset  = calibrateCurrentOffset(PV_CURRENT_CH);

  lcdDC.clear();
  lcdDC.setCursor(0, 0);
  lcdDC.print("DC Calibrated");
  delay(1000);

  pinMode(WATANI_RELAY_PIN, OUTPUT); 
  pinMode(INVERTER_RELAY_PIN, OUTPUT);
  pinMode(INV_INPUT_CONTACTOR_PIN, OUTPUT);
  pinMode(ESSENTIAL_LOAD_PIN, OUTPUT);
  pinMode(NON_ESSENTIAL_LOAD_PIN, OUTPUT);

  digitalWrite(WATANI_RELAY_PIN, HIGH); 
  digitalWrite(INVERTER_RELAY_PIN, LOW); 
  digitalWrite(INV_INPUT_CONTACTOR_PIN, HIGH);

  isWataniActive = false; 

  lcdDC.clear();
  lcdAC.clear();

  lcdDC.setCursor(0,0); 
  lcdDC.print("Welcome to      ");
  lcdDC.setCursor(0,1); 
  lcdDC.print("Hybrid PV System");

  lcdAC.setCursor(0,0); 
  lcdAC.print("ENG.Ali Moaz    ");
  lcdAC.setCursor(0,1); 
  lcdAC.print("ENG.Redha Sabah ");

  delay(3000); 

  lcdAC.clear(); 
  lcdDC.clear();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(1000);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();
  }

  if (millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();

    if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
      connectToBlynkNonBlocking();
    }
  }

  if (millis() - prevMillis > 1000) {
    prevMillis = millis();

    static int refreshCounter = 0;
    if (++refreshCounter >= 5) { 
      lcdAC.init(); 
      lcdAC.backlight();
      lcdDC.init(); 
      lcdDC.backlight();
      refreshCounter = 0; 
    }

    v1 = pzemGrid.voltage(); 
    i1 = pzemGrid.current(); 
    p1 = pzemGrid.power(); 
    f1 = pzemGrid.frequency(); 
    pf1 = pzemGrid.pf();

    v2 = pzemInv.voltage(); 
    i2 = pzemInv.current(); 
    p2 = pzemInv.power(); 
    f2 = pzemInv.frequency(); 
    pf2 = pzemInv.pf();

    if (isnan(v1)) v1 = 0;
    if (isnan(i1)) i1 = 0;
    if (isnan(p1)) p1 = 0;
    if (isnan(f1)) f1 = 0;
    if (isnan(pf1)) pf1 = 0;

    if (isnan(v2)) v2 = 0;
    if (isnan(i2)) i2 = 0;
    if (isnan(p2)) p2 = 0;
    if (isnan(f2)) f2 = 0;
    if (isnan(pf2)) pf2 = 0;

    // تنظيف قراءات الوطني إذا كان مطفي أو غير صالح
    if (v1 < 50.0) {
      v1 = 0;
      i1 = 0;
      p1 = 0;
      f1 = 0;
      pf1 = 0;
    }

    // تنظيف قراءات الإنفرتر إذا كان مطفي أو غير صالح
    if (v2 < 50.0) {
      v2 = 0;
      i2 = 0;
      p2 = 0;
      f2 = 0;
      pf2 = 0;
    }

    pvV  = readADSVoltageAverage(PV_VOLT_CH)  * PV_VOLT_FACTOR;
    batV = readADSVoltageAverage(BAT_VOLT_CH) * BAT_VOLT_FACTOR;

    batI = (readADSVoltageAverage(BAT_CURRENT_CH) - batCurrentOffset) / ACS_SENS;
    pvI  = (readADSVoltageAverage(PV_CURRENT_CH)  - pvCurrentOffset) / ACS_SENS;

    if (pvV < 1.0) pvV = 0;
    if (batV < 1.0) batV = 0;

    if (abs(batI) < CURRENT_ZERO_LIMIT) batI = 0;
    if (abs(pvI) < CURRENT_ZERO_LIMIT) pvI = 0;

    if (pvV == 0) pvI = 0;
    if (batV == 0) batI = 0;

    batSOC = calculateSoC(batV);

    batPowerW = batV * abs(batI);
    pvPowerW  = pvV  * abs(pvI);

    bool gridAvailable = (v1 > 175.0);
    bool pvCanCharge = (pvV > 13.5 && abs(pvI) > 0.3 && pvPowerW > 10);

    if (!manualMode) {
      if (batSOC >= 100) {
        digitalWrite(INV_INPUT_CONTACTOR_PIN, HIGH);

        if (!batteryFullNotified) {
          sendEvent("battery_full", "Battery reached 100%. Grid charging has been disconnected.");
          batteryFullNotified = true;
        }
      }
      else if (pvCanCharge) {
        digitalWrite(INV_INPUT_CONTACTOR_PIN, HIGH);
      }
      else {
        if (gridAvailable && batSOC < 80) {
          digitalWrite(INV_INPUT_CONTACTOR_PIN, LOW);
        }
        else {
          digitalWrite(INV_INPUT_CONTACTOR_PIN, HIGH);
        }
      }
    }

    if (batSOC < 95) {
      batteryFullNotified = false;
    }

    bool gridCharging = (gridAvailable && digitalRead(INV_INPUT_CONTACTOR_PIN) == LOW);

    if (!manualMode && !isSwitching) {
      if (v1 > 175.0) {
        if (!isWataniActive) {
          gridStableCounter++;
          if (gridStableCounter >= 5) { 
            safeSwitchToGrid(); 
          }
        }
      } 
      else if (v1 < 160.0) {
        gridStableCounter = 0;
        if (isWataniActive) {
          safeSwitchToInv();
        }
      }
    }

    if (manualMode) {
      digitalWrite(ESSENTIAL_LOAD_PIN, v15_state ? LOW : HIGH); 
      digitalWrite(NON_ESSENTIAL_LOAD_PIN, v14_state ? HIGH : LOW);

      if (v15_state && v14_state) loadStatus = "FULL";
      else if (v15_state || v14_state) loadStatus = "HALF";
      else loadStatus = "OFF ";
    } 
    else {
      if (isWataniActive) {
        digitalWrite(ESSENTIAL_LOAD_PIN, LOW); 
        digitalWrite(NON_ESSENTIAL_LOAD_PIN, HIGH);
        loadStatus = "FULL";
      } 
      else {
        bool pvAvailable = (pvV > 5.0);

        if (pvAvailable) {
          digitalWrite(ESSENTIAL_LOAD_PIN, LOW);
          digitalWrite(NON_ESSENTIAL_LOAD_PIN, HIGH);
          loadStatus = "FULL";
        } 
        else {
          if (batSOC > 50) {
            digitalWrite(ESSENTIAL_LOAD_PIN, LOW);
            digitalWrite(NON_ESSENTIAL_LOAD_PIN, HIGH);
            loadStatus = "FULL";
          } 
          else if (batSOC > 10) {
            digitalWrite(ESSENTIAL_LOAD_PIN, LOW);
            digitalWrite(NON_ESSENTIAL_LOAD_PIN, LOW);
            loadStatus = "HALF";

            if (!halfLoadNotified) {
              sendEvent("battery_half_load", "Battery below 50%. Non-essential loads turned OFF.");
              halfLoadNotified = true;
            }
          } 
          else {
            digitalWrite(ESSENTIAL_LOAD_PIN, HIGH);
            digitalWrite(NON_ESSENTIAL_LOAD_PIN, LOW);
            loadStatus = "OFF ";

            if (!allOffNotified) {
              sendEvent("battery_all_off", "Battery reached 10%. All loads have been turned OFF.");
              allOffNotified = true;
            }
          }
        }
      }
    }

    if (batSOC <= 20 && batSOC > 10 && !battery20Notified) {
      sendEvent("battery_20_warning", "Battery reached 20%. System shutdown is approaching.");
      battery20Notified = true;
    }

    if (batSOC > 55) halfLoadNotified = false;
    if (batSOC > 25) battery20Notified = false;
    if (batSOC > 15) allOffNotified = false;

    if (!isSwitching) {
      lcdAC.setCursor(0, 0);

      if (isWataniActive) { 
        lcdAC.print("GRID");

        if (gridCharging) {
          lcdAC.print("(CHG)");
        } 
        else {
          lcdAC.print("     ");
        }
      } 
      else { 
        if (gridStableCounter > 0) {
          lcdAC.print("GRID CHECK ");
          lcdAC.print(gridStableCounter);
          lcdAC.print(" ");
        } 
        else {
          lcdAC.print("INV SYSTEM ");
        }
      }

      lcdAC.setCursor(13, 0); 
      lcdAC.print(manualMode ? "MAN" : "AUT");

      lcdAC.setCursor(0, 1);

      float cV = isWataniActive ? v1 : v2; 
      float cA = isWataniActive ? i1 : i2; 
      float cW = isWataniActive ? p1 : p2;

      lcdAC.print("V=");
      lcdAC.print((int)cV); 
      lcdAC.print(" A=");
      lcdAC.print(cA, 1); 
      lcdAC.print(" W=");
      lcdAC.print((int)cW);
    }

    lcdDC.setCursor(0, 0); 
    lcdDC.print("PV:");
    lcdDC.print(pvV > 1.0 ? "ON " : "OFF");

    lcdDC.setCursor(9, 0); 
    lcdDC.print("L:");
    lcdDC.print(loadStatus);

    lcdDC.setCursor(0, 1); 
    lcdDC.print(String(batV, 1));
    lcdDC.print("V ");

    lcdDC.setCursor(7, 1); 
    lcdDC.print("SoC:");
    lcdDC.print(batSOC);
    lcdDC.print("% ");

    lcdDC.setCursor(15, 1); 
    lcdDC.print(gridCharging ? "C" : " ");

    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
      Blynk.virtualWrite(V0, v1); 
      Blynk.virtualWrite(V1, i1);
      Blynk.virtualWrite(V2, p1); 
      Blynk.virtualWrite(V3, f1);
      Blynk.virtualWrite(V4, pf1);

      Blynk.virtualWrite(V8, v2); 
      Blynk.virtualWrite(V9, i2);
      Blynk.virtualWrite(V12, p2); 
      Blynk.virtualWrite(V10, f2);
      Blynk.virtualWrite(V11, pf2);

      Blynk.virtualWrite(V17, batV);
      Blynk.virtualWrite(V18, batI);
      Blynk.virtualWrite(V19, batPowerW);

      Blynk.virtualWrite(V20, pvV);
      Blynk.virtualWrite(V21, pvI);
      Blynk.virtualWrite(V22, pvPowerW);

      Blynk.virtualWrite(V23, batSOC);
    }
  }
}