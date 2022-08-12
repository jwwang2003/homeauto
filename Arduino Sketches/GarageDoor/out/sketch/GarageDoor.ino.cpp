#include <Arduino.h>
#line 1 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
/**
 * @file GarageDoor.ino
 * @author Jimmy Wang
 * @brief Garage door controller
 * @version 1.0
 * @date 2022-08-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define SKETCH_NAME "Garage Door"
#define SKETCH_VERSION "1.0"

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

// #define G_DOOR_1_PIN 32
// #define G_DOOR_2_PIN 33
// #define G_DOOR_3_PIN 25
// #define G_DOOR_4_PIN 26

#define G_DOOR_1_SENS_PIN 13
#define G_DOOR_2_SENS_PIN 14

#define ATHXX_SAMPLERATE 10000 // Temperature and humidity sample rate
#define REED_SENSOR_SAMPLERATE 100

#define G_DOOR_OPEN_TIME 13*1000  // 12s to fully open
#define G_DOOR_CLOSE_TIME 17*1000 // 15s to fully close

#include "Vector.h"
#include "cppQueue.h"
#include <MySensors.h>
#include <Wire.h>
#include "AHTxx.h"
#include "SimpleTimer.h"

#define ELEMENT_COUNT_MAX 100

typedef struct Trigger {
  int intervalStages[3] = { 50, 400, 750 };

  uint8_t stage;
  uint8_t pin;
  uint16_t start;
  uint8_t it; // number of iterations left

  Trigger(uint8_t pin, uint8_t it): stage(0), pin(pin), start(millis()), it(it) {}

  bool work() {
    uint16_t curTime = millis();
    if(start + intervalStages[stage] <= curTime) {
      bool eval = stage == 0 || stage == 2;
      digitalWrite(pin, eval ? HIGH : LOW);

      // move on to next stage
      stage++;

      // stages finished and final iteration, return true to show this worker has finished
      if(stage >= 3 && it == 0) return true;
      else if(stage >= 3 && it > 0) {
        // stages finished but n more iterations to go
        stage = 0;
        it--;

        if(it == 0) return true;
      }

      start = curTime;
    }
    return false;
  }
};

Trigger* triggerArray[ELEMENT_COUNT_MAX];
Vector<Trigger*> triggerVector(triggerArray);

void handleGarageTimeoutCallback();
void handleSensorCheckCallback();
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

    // class constructor
    GarageDoor(int statusID, int textID, int outputPin, int inputPin) {
      this->statusID = statusID;
      this->textID = textID;
      this->outputPin = outputPin;
      this->inputPin = inputPin;

      this->doorState = doorClosed;
      this->senseState = senClosed;

      reportGarageDoorSwitch = new MyMessage(statusID, V_STATUS);
      reportGarageDoorState = new MyMessage(textID, V_TEXT);

      garageTimerId = -1;
      garageTimerTimestamp = 0;
    }

    void init() {
      this->readSensorState();
      if(isSensorClosed()) updateState(doorClosed);
      else updateState(doorOpened);
    }

    void reportData() {
      send(this->reportGarageDoorSwitch->set(doorState != doorClosed && doorState != doorClosing));
      send(this->reportGarageDoorState->set(this->doorState));
    }
    
    // called by recieve function
    // handles user input from HA
    void handleStateChange(bool newState) {
      newState ? this->open(): this->close();
      // reportData();
    }

    // called periodically by timer callback
    void handleSensorCheck() {
      if(!this->readSensorState()) return;

      if(this->isSensorClosed()) {
        if(doorState == doorClosing)
          confirmClose();
        else
          // door closed unexpectedly, normal?
          updateState(doorClosed);
      } else {
        if(doorState == doorClosed) {
          // WARNING - Garage door sensor detected unintentional opening
          updateState(doorOpened);
        }
        if(doorState == doorClosing || doorState == doorOpening) {
          // normal
        } else {
          // still normal?
        }
      }
    }

    // called by timer callback
    // if time exceeded then something might have gone wrong
    void handleTimeout() {
      if(doorState == doorClosed || doorState == doorOpened) return;

      int tLim = doorState == doorClosing ? G_DOOR_CLOSE_TIME : G_DOOR_OPEN_TIME;
      Serial.print(garageTimerTimestamp + tLim);
      Serial.print(" ");
      Serial.println(millis());
      if(garageTimerTimestamp + tLim < millis()) return;

      if(doorState == doorClosing) {
        updateState(doorHalted);
      } else if(doorState == doorOpening) {
        updateState(doorOpened);
      }

      this->cleanup();
    }

    // getter methods
    int getDoorState() { return doorState; }
    int getSensorState() { return senseState; }

    private:
      enum garageDoorStates {
        doorClosed,
        doorOpened,
        doorClosing,
        doorOpening,
        doorHalted,
      } doorState;

      enum sensorStates {
        senOpened,
        senClosed,
      } senseState;

      MyMessage *reportGarageDoorSwitch;
      MyMessage *reportGarageDoorState;

      int8_t garageTimerId;
      uint32_t garageTimerTimestamp;

      // door controls
      bool open() {
        garageTimerId = timer.setTimeout(G_DOOR_OPEN_TIME, handleGarageTimeoutCallback);
        garageTimerTimestamp = millis();
        switch(doorState) {
          case 0:
            this->toggle();
            break;
          case 2:
            this->toggle();
            break;
          default:
            return false;
        }
        return true;
      }

      // door controls
      bool close() {
        garageTimerId = timer.setTimeout(G_DOOR_CLOSE_TIME, handleGarageTimeoutCallback);
        garageTimerTimestamp = millis();
        switch(doorState) {
          case 1:
            this->toggle();
            break;
          case 3:
            this->toggle(2);
            break;
          case 4:
            this->toggle();
            break;
          default:
            return false;
        }
        return true;
      }

      void updateState(int newState) {
        doorState = static_cast<garageDoorStates>(newState);
        this->reportData();
      }

      bool isClosed() {
        // if closed or closing
        return doorState == doorClosed || doorClosing;
      }

      bool isSensorClosed() {
        this->readSensorState();

        if(this->senseState == senClosed) return true;
        else return false;
      }

      bool readSensorState() {
        int ATM = digitalRead(this->inputPin); // 1 - Closed, 0 - Open

        sensorStates atm = static_cast<sensorStates>(ATM);

        if(atm == this->senseState && this->senseState != NULL) return false;
        
        this->senseState = atm;
        return true;
      }

      void confirmClose() {
        updateState(0);

        this->cleanup();
      }

      /**
       * @brief This function emulates a button press on the actual garage door remote
       * 
       * @param it the number of times to press
       */
      void toggle(int it = 1) {
        for(int i = 0; i < it; ++i) {
          if(doorState == doorClosed) this->updateState(doorOpening);
          else if(doorState == doorOpened) this->updateState(doorClosing);
          else if(doorState == doorClosing) this->updateState(doorOpening);
          else if(doorState == doorOpening) this->updateState(doorHalted);
          else if(doorState == doorHalted) this->updateState(doorClosing);
        }

        triggerVector.push_back(new Trigger(outputPin, it));

        /*
        // first time toggle no need to pause
        if(i != 0) delay(750);
        digitalWrite(outputPin, HIGH);
        delay(50);
        digitalWrite(outputPin, LOW);
        delay(400);
        digitalWrite(outputPin, HIGH);
        */
      }

      void cleanup() {
        if(garageTimerId != -1) {
          timer.disable(garageTimerId);
          timer.deleteTimer(garageTimerId);
          garageTimerId = -1;
        }
        if(garageTimerTimestamp != 0) {
          garageTimerTimestamp = 0;
        }
      }
};

GarageDoor *garageDoors[3] = {
  new GarageDoor(CHILD_ID_G_DOOR_1, CHILD_ID_G_DOOR_1_STATUS, G_DOOR_1_PIN, G_DOOR_1_SENS_PIN),
  NULL,
  new GarageDoor(CHILD_ID_G_DOOR_2, CHILD_ID_G_DOOR_2_STATUS, G_DOOR_2_PIN, G_DOOR_2_SENS_PIN)
};

#line 369 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void presentation();
#line 382 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void setup();
#line 418 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void loop();
#line 428 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void receive(const MyMessage &msg);
#line 359 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void handleGarageTimeoutCallback() {
  garageDoors[0]->handleTimeout();
  garageDoors[2]->handleTimeout();
}

void handleSensorCheckCallback() {
  garageDoors[0]->handleSensorCheck();
  garageDoors[2]->handleSensorCheck();
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

  timer.setInterval(REED_SENSOR_SAMPLERATE, handleSensorCheckCallback);
}

void loop() {
  timer.run();
  for(int i = 0; i < triggerVector.size(); ++i) {
    if(triggerVector[i]->work()) {
      delete triggerVector[i];
      triggerVector.remove(i);
    }
  }
}

void receive(const MyMessage &msg) {
  // message is coming from the gateway, safety checks, and error "handling"
  if(msg.sender == 0) {
    switch(msg.type) {
      case V_STATUS:
        if(msg.sensor == 0 || msg.sensor == 2) {
          try {
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
