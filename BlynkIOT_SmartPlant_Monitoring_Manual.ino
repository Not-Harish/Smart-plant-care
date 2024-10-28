// Viral Science www.viralsciencecreativity.com www.youtube.com/c/viralscience
// Blynk IOT Smart Plant Monitoring System

/* Connections
Relay. GPIO23
Soil.  GPIO33
PIR.   GPIO19
SDA.   GPIO21
SCL.   GPIO22
Temp.  GPIO18
*/

// Include the library files
#include <LiquidCrystal_I2C.h>
#define BLYNK_PRINT Serial
#include <WiFi.h>              // Use WiFi.h for ESP32
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// Initialize the LCD display
LiquidCrystal_I2C lcd(0x3F, 16, 2);

char auth[] = "T8_wchJWx2MNEtaqTMnFKqXDYbVCBAY2";  // Enter your Blynk Auth token
char ssid[] = "Redmi Note 10";  // Enter your WIFI SSID
char pass[] = "12345678";  // Enter your WIFI Password

DHT dht(18, DHT11); // (DHT sensor pin, sensor type) GPIO18 for ESP32 DHT11 Temperature Sensor
BlynkTimer timer;

// Define component pins
#define soil 33     // GPIO33 Soil Moisture Sensor (Analog Input)
#define PIR 19      // GPIO19 PIR Motion Sensor
#define RELAY_PIN_1 23 // GPIO23 for Relay Control

int PIR_ToggleValue;
int relay1State = LOW;

// Create three variables for pressure
double T, P;
char status;

void setup() {
  Serial.begin(115200);       // Higher baud rate for ESP32
  lcd.begin();
  lcd.backlight();
  pinMode(PIR, INPUT);

  pinMode(RELAY_PIN_1, OUTPUT);
  digitalWrite(RELAY_PIN_1, LOW);

  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  dht.begin();

  lcd.setCursor(0, 0);
  lcd.print("  Initializing  ");
  for (int a = 5; a <= 10; a++) {
    lcd.setCursor(a, 1);
    lcd.print(".");
    delay(500);
  }
  lcd.clear();

  // Call the functions
  timer.setInterval(100L, soilMoistureSensor);
  timer.setInterval(100L, DHT11sensor);
}

// Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t);

  lcd.setCursor(8, 0);
  lcd.print("H:");
  lcd.print(h);
}

// Get the soil moisture values
void soilMoistureSensor() {
  int value = analogRead(soil);
  value = map(value, 0, 4095, 0, 100);  // Adjusted for ESP32 (0-4095)
  value = (value - 100) * -1;

  Blynk.virtualWrite(V3, value);
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(value);
  lcd.print(" ");
}

// Get the PIR sensor values
void PIRsensor() {
  bool value = digitalRead(PIR);
  if (value) {
    Blynk.logEvent("pirmotion", "WARNING! Motion Detected!");
    WidgetLED LED(V5);
    LED.on();
  } else {
    WidgetLED LED(V5);
    LED.off();
  }
}

BLYNK_WRITE(V6) {
  PIR_ToggleValue = param.asInt();
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V6);
}

void loop() {
  if (PIR_ToggleValue == 1) {
    lcd.setCursor(5, 1);
    lcd.print("M:ON ");
    PIRsensor();
  } else {
    lcd.setCursor(5, 1);
    lcd.print("M:OFF");
    WidgetLED LED(V5);
    LED.off();
  }

  Blynk.run();  // Run the Blynk library
  timer.run();  // Run the Blynk timer
}
