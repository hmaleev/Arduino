/* 
Connect the photoresistor one leg to pin 0, and pin to +5V
Connect a resistor ( 10k is a good value, higher
values gives higher readings) from pin 0 to GND.
----------------------------------------------------
           PhotoR     10K
 +5    o---/\/\/--.--/\/\/---o GND
                  |
 Pin 0 o-----------
----------------------------------------------------
*/
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SPI.h>
#include <MFRC522.h>

int lightPin = 0;   // define a pin for Photo resistor
int ledPin= 13;     // define a pin for LED
int laserPin = 4;   // define a pin for the laser module

String dataToSend =  String(20);                         // string for data to send

SoftwareSerial BT(6, 7);    // RX, TX

#define DHTPIN            2         // Pin connected to the DHT sensor.
#define DHTTYPE           DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
sensor_t sensor;
sensors_event_t event;

constexpr uint8_t RST_PIN = 9;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 10;     // Configurable, see typical pin layout above
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

int maxCounter = 1;
int counter = 0;
          
void setup()
{
    Serial.begin(9600);  //Begin serial communcation
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    SPI.begin();                                                  // Init SPI bus
    mfrc522.PCD_Init();                                           // Init MFRC522 card
    
    // inits the software serial port 
    BT.begin(9600);

    // inits the temperature sensor
    dht.begin();
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
  

    pinMode(ledPin, OUTPUT);
    pinMode(laserPin, OUTPUT);
    dataToSend = "---|";
    digitalWrite(4, HIGH); 
}

void loop()
{

  if(dataToSend.length()>=6){
    dataToSend = "---|";
  }

//------------------------RFID -------------------
    MFRC522::MIFARE_Key key;
    for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

    //some variables we need
    byte block;
    byte len;
    MFRC522::StatusCode status;
 
    if (mfrc522.PICC_IsNewCardPresent()) {
        if (mfrc522.PICC_ReadCardSerial()) {

            byte buffer1[18];
            block = 4;
            len = 18;

            //------- GET Data
            status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); 
            if (status != MFRC522::STATUS_OK) { 
                return;
            }

            status = mfrc522.MIFARE_Read(block, buffer1, &len);
            if (status != MFRC522::STATUS_OK) {
                return;
            }

            //PRINT FIRST NAME
            String userID =  String();
            for (uint8_t i = 0; i < 16; i++)
            {
                if (buffer1[i] != 32) {
                   
                    char character = buffer1[i];
                    userID += String(character);
                }
            }
            
            dataToSend = userID+"|";
            if(userID == "420"){
              Serial.println("ACCESS GRANTED");
              // SEND SIGNAL TO UNLOCK DOOR FOR 10 SECONDS
            } else {
               Serial.println("ACCESS DENIED");
              // DONT SEND SIGNAL TO OPEN DOOR
              // START ALARM
            }
            
            mfrc522.PICC_HaltA();
            mfrc522.PCD_StopCrypto1();
        }
    }


//------------------------RFID ------------------

//------------------------LASER DETECTION START ------------------
    int laserData = analogRead(lightPin);
    if (laserData < 900) {
        //Serial.println("Laser stopped");
        digitalWrite(ledPin, HIGH);
        dataToSend += "0|";

    } else {
        // Serial.println("Laser Working");
        digitalWrite(ledPin, LOW);
        dataToSend += "1|";
    }
//------------------------LASER DETECTION END ------------------

//------------------------AIR PARAMS DETECTION START ------------------

     delay(delayMS);

      float temperature = 0.0;
      dht.temperature().getEvent(&event);
      if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
      }
      else {
        temperature = event.temperature;
      }
      
      float humidity = 0.0;
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
      }
      else {
       humidity = event.relative_humidity;
      }
//------------------------AIR PARAMS DETECTION END ------------------

    //Write the value of the photoresistor to the serial monitor.
    //Serial.println(analogRead(lightPin));
    delay(50); //shorter delay for faster response to light.

//------------------------BLUETOOTH COMMUNICATION  START ------------------
//    if(counter == maxCounter){
      counter=0;
      dataToSend +=  String(temperature, 2) +"|";
    
    dataToSend +=  String(humidity, 2) +"|";
    
      Serial.println(dataToSend);
      Serial.println();
      BT.print(dataToSend);
      BT.println("");
//    }
//------------------------BLUETOOTH COMMUNICATION  END ------------------

    counter++;
}
