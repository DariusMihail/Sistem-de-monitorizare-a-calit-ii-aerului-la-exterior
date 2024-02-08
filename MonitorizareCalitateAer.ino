#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include "Adafruit_SGP30.h"
#include <SoftwareSerial.h>
#include <WiFi.h>

// Define pin connections for sensors
#define DHTPIN 15
#define PMS_RX 16
#define PMS_TX 17

// Initialize sensors
DHT dht(DHTPIN, DHT22);
SoftwareSerial pmsSerial(PMS_RX, PMS_TX);
Adafruit_SGP30 sgp30;

// Replace with your network credentials
const char* ssid = "AndroidAP0726";
const char* password = "lulm7592";
const char* server = "api.thingspeak.com";

// Replace with your ThingSpeak channel ID and API key
unsigned long channelID = 2087293;
const char* apiKey = "EXDRVJ3A54RQNAZM";
const unsigned long updateInterval = 2000; // 15 seconds

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize variables for sensor readings
float temperature;
float humidity;
float co2;
float voc;
int pm25;
int pm10;

// Initialize timer for sending data to ThingSpeak
unsigned long lastUpdate = 0;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize DHT22 sensor
  dht.begin();

  // Initialize PMS5003 sensor
  pmsSerial.begin(9600);

  // Initialize SGP30 sensor
  if (! sgp30.begin()){
    Serial.println("SGP30 sensor not detected");
    while (1);
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Initialize LCD display
  lcd.init();
  lcd.backlight();

  // Clear LCD display
  lcd.clear();
}

void loop() {
  // Read temperature and humidity from DHT22 sensor
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Read CO2 and TVOC from SGP30 sensor
  if (! sgp30.IAQmeasure()) {
    Serial.println("SGP30 measurement failed");
    return;
  }
  co2 = sgp30.eCO2;
  voc = sgp30.TVOC;

  // Read PM2.5 and PM10 from PMS5003 sensor
  if (pmsSerial.available()) {
    // Read data from PMS5003 sensor
    unsigned char buf[32];
    int len = 0;
    while (pmsSerial.available()) {
      buf[len++] = pmsSerial.read();
      if (len == 32) break;
    }
    if (len == 32) {
      // Calculate PM2.5 and PM10 from sensor data
      pm25 = (buf[10]<<8) | buf[11];
      pm10 = (buf[12]<<8) | buf[13];
    }
  }

  // Update LCD display
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(9, 0);
  lcd.print("H:");
  lcd.print(humidity);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("CO2:");
  lcd.print(co2);
  lcd.print("ppm ");
  lcd.setCursor(10, 1);

  // aici am adaugat ce e mai jos (modific aici era aici)
  lcd.print("PM2.5:");
  lcd.print(pm25);
  lcd.print("ug/m3 ");
  // modific aici
  lcd.print("TVOC:");
  lcd.print(voc);
  lcd.print("ppb");

  //adaugat dupa
  // Display PM2.5 and PM10 on LCD
  lcd.setCursor(0, 1);
  lcd.print("PM2.5:");
  lcd.print(pm25);
  lcd.print("ug/m3 ");
  lcd.setCursor(10, 1);
  lcd.print("PM10:");
  lcd.print(pm10);
  lcd.print("ug/m3");


  // Send data to ThingSpeak
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;

    // Create ThingSpeak client
    WiFiClient client;
    if (!client.connect(server, 80)) {
      Serial.println("Failed to connect to ThingSpeak server");
      return;
    }

    // Create API request string with sensor data
    String apiString = "GET /update?api_key=";
    apiString += apiKey;
    apiString += "&field1=";
    apiString += String(temperature);
    apiString += "&field2=";
    apiString += String(humidity);
    apiString += "&field3=";
    apiString += String(co2);
    apiString += "&field4=";
    apiString += String(voc);
    apiString += "&field5=";
    apiString += String(pm25);
    apiString += "&field6=";
    apiString += String(pm10);
    apiString += " HTTP/1.1\r\n";
    apiString += "Host: ";
    apiString += server;
    apiString += "\r\n";
    apiString += "Connection: close\r\n\r\n";

    // Send API request to ThingSpeak server
    client.print(apiString);
    delay(500); // Wait for response from server

    // Print response from server to serial monitor
    while (client.available()) {
      Serial.write(client.read());
    }
    Serial.println();
  }
}

