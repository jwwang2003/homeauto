#define SKETCH_NAME "Garage Door"
#define SKETCH_VERSION "1.0.0"

// Enable debug prints
#define MY_DEBUG

/**
 * Radio wiring ESP32(Node32s): RF24, RFM69, RFM95:
 *
 * | IO   | RF24 | RFM69 | RFM95 |
 * |------|------|-------|-------|
 * | MOSI | 23   | 23    | 23    |
 * | MISO | 19   | 19    | 19    |
 * | SCK  | 18   | 18    | 18    |
 * | CSN  | 5    | 5     | 5     |
 * | CE   | 17   | -     | -     |
 * | RST  | -    | GND   | GND   |
 * | IRQ  | 16*  | 16    | 16    |
 * * = optional 
*/

#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_IRQ_PIN 16
#define MY_RFM69_IRQ_NUM MY_RFM69_IRQ_PIN
#define MY_RFM69_CS_PIN 5

#define CHILD_ID_G_DOOR_1 0
#define CHILD_ID_G_DOOR_2 1
// #define CHILD_ID_G_DOOR_3 2
// #define CHILD_ID_G_DOOR_4 3
#define CHILD_ID_G_HUM 4
#define CHILD_ID_G_TEMP 5

#define G_DOOR_1_PIN 25
#define G_DOOR_2_PIN 26
// #define G_DOOR_3_PIN 32
// #define G_DOOR_4_PIN 33

#define ATHXX_INTERVAL 10000 // Temperature and humidity reading interval

#include <MySensors.h>
#include <Wire.h>
#include "AHTxx.h"
#include "SimpleTimer.h"

// Initialize AHT10 sensor
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);

MyMessage reportHum(CHILD_ID_G_HUM, V_HUM);
MyMessage reportTemp(CHILD_ID_G_TEMP, V_TEMP);

void presentation() {
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);

  present(CHILD_ID_G_HUM, S_HUM, "车库湿度");
  present(CHILD_ID_G_TEMP, S_TEMP, "车库温度");

  present(CHILD_ID_G_DOOR_1, S_BINARY, "车库门1");
  present(CHILD_ID_G_DOOR_2, S_BINARY, "车库门2");
  // present(CHILD_ID_G_DOOR_3, S_BINARY);
  // present(CHILD_ID_G_DOOR_4, S_BINARY);

  // getControllerConfig().time()

  // metric = getControllerConfig().isMetric;
}

void toggle() {
  digitalWrite(G_DOOR_1_PIN, HIGH);
  delay(50);
  digitalWrite(G_DOOR_1_PIN, LOW);
  delay(500);
  digitalWrite(G_DOOR_1_PIN, HIGH);
}

void setup() {
  Wire.begin();

  I2C_Scanner();

  pinMode(G_DOOR_1_PIN, OUTPUT);
  pinMode(G_DOOR_2_PIN, OUTPUT);
  digitalWrite(G_DOOR_1_PIN, HIGH);
  digitalWrite(G_DOOR_2_PIN, HIGH);
  // pinMode(G_DOOR_3_PIN, OUTPUT);
  // pinMode(G_DOOR_4_PIN, OUTPUT);

  Serial.println(F("Beginning to initialize AHT10 sensor"));
  while(aht10.begin() != true) {
    Serial.println(F("AHT1x not connected or failed to load calibration coefficient"));

    delay(5000);
  }
  Serial.println(F("AHT10 OK"));

  toggle();

  delay(10000); // pause for 10 seconds
}

SimpleTimer AHTxx_Timer(ATHXX_INTERVAL);

void loop() {
  if(AHTxx_Timer.isReady()) {
    reportAHTxx();
    AHTxx_Timer.reset(); 
  }
}

void receive(const MyMessage &msg) {
  // CHILD SENSOR ID | msg.sensor

  switch(msg.sensor) {
    case 0 ... 1:
      // if the sensor destination is one of the garage door toggles
      Serial.println("Trigger");
      break;
    default:
      break;
  }

}

float ahtTemp, ahtHum;
/**
 * Reads data from AHT10 sensor (temperature and humidity) and send to gateway
 * 
 * @param
 * @return
 */
void reportAHTxx() {
  ahtTemp = aht10.readTemperature();

  if(ahtTemp != AHTXX_ERROR) {
    send(reportTemp.set(ahtTemp, 1));
  } else {
    Serial.println(getAHTxxStatus());
  }

  ahtHum = aht10.readHumidity(AHTXX_USE_READ_DATA);

  if(ahtHum != AHTXX_ERROR) {
    send(reportHum.set(ahtHum, 1));
  } else {
    Serial.println(getAHTxxStatus());
  }
}

/**
 * Converts ATHxx error code into a readble string and returns it
 * 
 * @param 
 * @return String a string describing the error
 */
String getAHTxxStatus() {
  switch(aht10.getStatus()) {
    case AHTXX_NO_ERROR:
      return F("no error");
      break;

    case AHTXX_BUSY_ERROR:
      return F("sensor busy, increase polling time");
      break;

    case AHTXX_ACK_ERROR:
      return F("sensor didn't return ACK, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)");
      break;

    case AHTXX_DATA_ERROR:
      return F("received data smaller than expected, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)");
      break;

    case AHTXX_CRC8_ERROR:
      return F("computed CRC8 not match received CRC8, this feature supported only by AHT2x sensors");
      break;

    default:
      return F("unknown status");    
      break;
  }
}

/**
 * Scans the I2C bus for devices and debugs them in the serial output
 * 
 * @param
 * @return
 */
void I2C_Scanner() {
  Serial.println();
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0) {  // Receive 0 = success (ACK response)
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);     // PCF8574 7 bit address
      Serial.println(")");
      count++;
    }
  }
  Serial.print("Found ");      
  Serial.print(count, DEC);        // numbers of devices
  Serial.println(" device(s).");
}