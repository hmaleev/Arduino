// Depends on the following Arduino libraries:
// - Adafruit Unified Sensor Library: https://github.com/adafruit/Adafruit_Sensor
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library

/*
 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)


*/

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>

#define DHTPIN            2         // Pin connected to the DHT sensor.
#define DHTTYPE           DHT21     // DHT 21 (AM2301)
SoftwareSerial BT(10, 11);    // RX, TX

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// Bluetooth module HC-06 is connected on pins 10(RX) and 11(TX). 
//This means that data passed to Serial.print() function is sent via bluetooth to other devices

void setup() {
  
  Serial.begin(9600); 
  // Initialize device.
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  BT.begin(9600);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void loop() {

  float temperature = 0.0;
  float humidity = 0.0;
  
  // Delay between measurements.
  delay(delayMS);

  sensors_event_t event;  
  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    temperature = event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
   humidity = event.relative_humidity;
  }

  Serial.print(temperature);
  Serial.print("");
  Serial.print("|");
  Serial.print(humidity);
  Serial.print("|");  
  Serial.println("");  

  
  BT.print(temperature);
  BT.print("");
  BT.print("|");
  BT.print(humidity);
  BT.print("|");
  BT.println(""); 
}

