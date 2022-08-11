/**
 * @file GarageDoor.ino
 * @author Jimmy Wang
 * @brief Garage door controller
 * @version 0.3
 * @date 2022-08-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define SKETCH_NAME "Garage Door"
#define SKETCH_VERSION "0.3"

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

// MySensors child IDs
#define CHILD_ID_G_DOOR_1 0
#define CHILD_ID_G_DOOR_1_STATUS 1
#define CHILD_ID_G_DOOR_2 2
#define CHILD_ID_G_DOOR_2_STATUS 3
// #define CHILD_ID_G_DOOR_3 2
// #define CHILD_ID_G_DOOR_4 3
#define CHILD_ID_G_HUM 4
#define CHILD_ID_G_TEMP 5

// Hardware input & output pins
#define G_DOOR_1_PIN 25
#define G_DOOR_2_PIN 26
#define G_DOOR_3_PIN 32
#define G_DOOR_4_PIN 33

#define G_DOOR_1_SENS_PIN 13
#define G_DOOR_2_SENS_PIN 14

#define ATHXX_SAMPLERATE 10000 // Temperature and humidity sample rate
#define REED_SENSOR_SAMPLERATE 100

#define G_DOOR_OPEN_TIME 12*1000  // 12s to fully open
#define G_DOOR_CLOSE_TIME 15*1000 // 15s to fully close

#include <MySensors.h>
#include <Wire.h>
#include "AHTxx.h"
#include "SimpleTimer.h"

void handleGarageDoorCallback();
void reportAHTxx();
String getAHTxxStatus();
void I2C_Scanner();

// Initialize AHT10 sensor
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);

MyMessage reportHum(CHILD_ID_G_HUM, V_HUM);
MyMessage reportTemp(CHILD_ID_G_TEMP, V_TEMP);

SimpleTimer timer;

int AHTxxTimerID;

class GarageDoor {
  public:
    int statusID;
    int textID;
    int outputPin;
    int inputPin;

    // initializer
    GarageDoor(int statusID, int textID, int outputPin, int inputPin) {
      this->statusID = statusID;
      this->textID = textID;
      this->outputPin = outputPin;
      this->inputPin = inputPin;

      sensorState = NULL;
      prevState = NULL;
      curState = NULL;

      reportGarageDoorSwitch = new MyMessage(statusID, V_STATUS);
      reportGarageDoorState = new MyMessage(textID, V_TEXT);

      garageTimerId = -1;
    }

    char *getStateString() { return garageDoorState[curState]; }
    char *getStateString(int s) { return garageDoorState[s]; }

    void init();
    bool isClosed();

    bool open();
    bool close();

    void reportData();
    void updateState(int newState);
    void handleTimeout();
    bool handleStateChange(bool newState);
    void checkSensorStateChange();
    bool isOpen();
    bool readSensorState();

  private:
    char *garageDoorState[6] = { 
      "关闭", 
      "打开", 
      "关闭中", 
      "打开中", 
      "静止",
      "警告"
    };

    int sensorState;
    int curState, prevState;

    MyMessage *reportGarageDoorSwitch;
    MyMessage *reportGarageDoorState;

    int garageTimerId;

    void toggle(int it = 1);
    void updateSensorState(int newState) { sensorState = newState; }
    void confirmClose();
};

void GarageDoor::init() {
  readSensorState();
  if(sensorState == 1) updateState(0);
  else updateState(1);
}

void GarageDoor::reportData() {
  send(reportGarageDoorSwitch->set(curState != 0 && curState != 2));
  send(reportGarageDoorState->set(getStateString(curState)));
}

void GarageDoor::updateState(int newState) {
  prevState = curState;
  curState = newState;
  reportData();
}

bool GarageDoor::open() {
  garageTimerId = timer.setTimeout(G_DOOR_OPEN_TIME, handleGarageDoorCallback);
  switch(curState) {
    case 0:
      toggle();
      break;
    case 2:
      toggle();
      break;
    default:
      return false;
  }
  return true;
}

bool GarageDoor::close() {
  garageTimerId = timer.setTimeout(G_DOOR_CLOSE_TIME, handleGarageDoorCallback);
  switch(curState) {
    case 1:
      toggle();
      break;
    case 3:
      toggle(2);
      break;
    case 4:
      toggle();
      break;
    default:
      return false;
  }
  return true;
}

void GarageDoor::confirmClose() {
  if(garageTimerId != -1) {
    timer.disable(garageTimerId);
    timer.deleteTimer(garageTimerId);
    garageTimerId = -1;
  }

  updateState(0);
}

void GarageDoor::handleTimeout() {
  if(curState == 0 || curState == 1) return;

  if(curState == 2) {
    updateState(4);
  } else if(curState == 3) {
    updateState(1);
  }
  
  garageTimerId = -1;
}

bool GarageDoor::handleStateChange(bool newState) {
  bool res = newState == 0 ? close() : open();
  reportData();
  return res;
}

void GarageDoor::checkSensorStateChange() {
  if(!readSensorState()) return;

  if(isClosed()) {
    if(curState == 2)
      confirmClose();
    else
      updateState(0);
  } else {
    if(curState == 0) {
      // WARNING - Garage door sensor detected unintentional opening
      updateState(1);
    }
    if(curState == 2 || curState == 3) {
      // normal
    } else {
      // still normal?
    }
  }
}

bool GarageDoor::isClosed() {
  readSensorState();

  if(sensorState == 1) return true;
  else return false;
}

bool GarageDoor::readSensorState() {
  int ATM = digitalRead(inputPin);

  if(ATM == sensorState && sensorState != NULL) return false;
  
  sensorState = ATM;
  return true;
}

void GarageDoor::toggle(int it) {
  for(; it > 0; --it) {
    if(curState == 0) updateState(3);
    else if(curState == 1) updateState(2);
    else if(curState == 2) updateState(3);
    else if(curState == 3) updateState(4);
    else if(curState == 4) updateState(2);

    digitalWrite(outputPin, HIGH);
    delay(50);
    digitalWrite(outputPin, LOW);
    delay(400);
    digitalWrite(outputPin, HIGH);
  }
}

GarageDoor *garageDoors[3] = {
  new GarageDoor(CHILD_ID_G_DOOR_1, CHILD_ID_G_DOOR_1_STATUS, G_DOOR_1_PIN, G_DOOR_1_SENS_PIN),
  NULL,
  new GarageDoor(CHILD_ID_G_DOOR_2, CHILD_ID_G_DOOR_2_STATUS, G_DOOR_2_PIN, G_DOOR_2_SENS_PIN)
};

void handleGarageDoorCallback() {
  garageDoors[0]->handleTimeout();
  garageDoors[2]->handleTimeout();
}

void handleCheckSensorCallback() {
  garageDoors[0]->checkSensorStateChange();
  garageDoors[2]->checkSensorStateChange();
}

void presentation() {
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);

  present(CHILD_ID_G_HUM, S_HUM, "车库湿度");
  present(CHILD_ID_G_TEMP, S_TEMP, "车库温度");

  present(CHILD_ID_G_DOOR_1, S_BINARY, "车库门 1");
  present(CHILD_ID_G_DOOR_1_STATUS, S_INFO, "车库门 1 状态");

  present(CHILD_ID_G_DOOR_2, S_BINARY, "车库门 2");
  present(CHILD_ID_G_DOOR_2_STATUS, S_INFO, "车库门 2 状态");
}

void setup() {
  Wire.begin();
  I2C_Scanner();

  pinMode(2, OUTPUT);
  
  pinMode(G_DOOR_1_PIN, OUTPUT);
  pinMode(G_DOOR_2_PIN, OUTPUT);
  pinMode(G_DOOR_3_PIN, OUTPUT);
  pinMode(G_DOOR_4_PIN, OUTPUT);
  pinMode(G_DOOR_1_SENS_PIN, INPUT);
  pinMode(G_DOOR_2_SENS_PIN, INPUT);

  // Set all garage remote output pins to OFF
  digitalWrite(G_DOOR_1_PIN, HIGH);
  digitalWrite(G_DOOR_2_PIN, HIGH);
  digitalWrite(G_DOOR_3_PIN, HIGH);
  digitalWrite(G_DOOR_4_PIN, HIGH);

  garageDoors[0]->init();
  garageDoors[2]->init();

  Serial.println(F("Beginning to initialize AHT10 sensor"));
  while(aht10.begin() != true) {
    Serial.println(F("AHT1x not connected or failed to load calibration coefficient"));
    delay(5000);
  }
  Serial.println(F("AHT10 OK"));

  delay(5000); // pause for 5 seconds

  AHTxxTimerID = timer.setInterval(ATHXX_SAMPLERATE, reportAHTxx);
  timer.enable(AHTxxTimerID);

  timer.setInterval(REED_SENSOR_SAMPLERATE, handleCheckSensorCallback);
}

void loop() {
  timer.run();
}

void receive(const MyMessage &msg) {
  // message is coming from the gateway, safety checks, and error "handling"
  if(msg.sender == 0) {
    switch(msg.type) {
      case V_STATUS:
        if(msg.sensor == 0 || msg.sensor == 2) {
          try {
            Serial.println((String)msg.sensor + " " + msg.data);
            garageDoors[msg.sensor]->handleStateChange(msg.getBool());
          } catch (int e) { Serial.println("An error occured\n" + e); }
        }
        break;
      default:
        break;
    }
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