# 1 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 15 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
// Enable debug prints



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
# 41 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
// MySensors child IDs




// #define CHILD_ID_G_DOOR_3 2
// #define CHILD_ID_G_DOOR_4 3



// Hardware input & output pins





// #define G_DOOR_1_PIN 32
// #define G_DOOR_2_PIN 33
// #define G_DOOR_3_PIN 25
// #define G_DOOR_4_PIN 26
# 71 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
# 72 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 73 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 74 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 75 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 76 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 77 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2



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
      digitalWrite(pin, eval ? 0x1 : 0x0);

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

Trigger* triggerArray[100];
Vector<Trigger*> triggerVector(triggerArray);

void handleGarageTimeoutCallback();
void handleSensorCheckCallback();
void reportAHTxx();
String getAHTxxStatus();
void I2C_Scanner();

// Initialize AHT10 sensor
AHTxx aht10(0x38 /*AHT15/AHT20/AHT21/AHT25 I2C address, AHT10 I2C address if address pin to GND*/, AHT1x_SENSOR);

MyMessage reportHum(4, V_HUM);
MyMessage reportTemp(5, V_TEMP);

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

      int tLim = doorState == doorClosing ? 17*1000 /* 15s to fully close*/ : 13*1000 /* 12s to fully open*/;
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
        garageTimerId = timer.setTimeout(13*1000 /* 12s to fully open*/, handleGarageTimeoutCallback);
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
        garageTimerId = timer.setTimeout(17*1000 /* 15s to fully close*/, handleGarageTimeoutCallback);
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

        if(atm == this->senseState && this->senseState != 
# 302 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
                                                         __null
# 302 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
                                                             ) return false;

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
# 319 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 339 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
  new GarageDoor(0, 1, 25, 13),
  
# 355 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
 __null
# 355 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
     ,
  new GarageDoor(2, 3, 26, 14)
};

void handleGarageTimeoutCallback() {
  garageDoors[0]->handleTimeout();
  garageDoors[2]->handleTimeout();
}

void handleSensorCheckCallback() {
  garageDoors[0]->handleSensorCheck();
  garageDoors[2]->handleSensorCheck();
}

void presentation() {
  sendSketchInfo("Garage Door", "1.0");

  present(4, S_HUM, "车库湿度");
  present(5, S_TEMP, "车库温度");

  present(0, S_BINARY, "车库门 1");
  present(1, S_INFO, "车库门 1 状态");

  present(2, S_BINARY, "车库门 2");
  present(3, S_INFO, "车库门 2 状态");
}

void setup() {
  Wire.begin();
  I2C_Scanner();

  pinMode(2, 0x03);

  pinMode(25, 0x03);
  pinMode(26, 0x03);
  pinMode(32, 0x03);
  pinMode(33, 0x03);
  pinMode(13, 0x01);
  pinMode(14, 0x01);

  // Set all garage remote output pins to OFF
  digitalWrite(25, 0x1);
  digitalWrite(26, 0x1);
  digitalWrite(32, 0x1);
  digitalWrite(33, 0x1);

  garageDoors[0]->init();
  garageDoors[2]->init();

  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Beginning to initialize AHT10 sensor")))));
  while(aht10.begin() != true) {
    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT1x not connected or failed to load calibration coefficient")))));
    delay(5000);
  }
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT10 OK")))));

  delay(5000); // pause for 5 seconds

  AHTxxTimerID = timer.setInterval(10000 /* Temperature and humidity sample rate*/, reportAHTxx);

  timer.setInterval(100, handleSensorCheckCallback);
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
# 452 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void reportAHTxx() {
  ahtTemp = aht10.readTemperature();

  if(ahtTemp != 0xFF /*other errors*/) {
    send(reportTemp.set(ahtTemp, 1));
  } else {
    Serial.println(getAHTxxStatus());
  }

  ahtHum = aht10.readHumidity(false /*force to use data from previous read*/);

  if(ahtHum != 0xFF /*other errors*/) {
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
# 476 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
String getAHTxxStatus() {
  switch(aht10.getStatus()) {
    case 0x00 /*success, no errors*/:
      return ((reinterpret_cast<const __FlashStringHelper *>(("no error"))));
      break;

    case 0x01 /*sensor is busy*/:
      return ((reinterpret_cast<const __FlashStringHelper *>(("sensor busy, increase polling time"))));
      break;

    case 0x02 /*sensor didn't return ACK (not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))*/:
      return ((reinterpret_cast<const __FlashStringHelper *>(("sensor didn't return ACK, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"))));
      break;

    case 0x03 /*received data smaller than expected*/:
      return ((reinterpret_cast<const __FlashStringHelper *>(("received data smaller than expected, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"))));
      break;

    case 0x04 /*computed CRC8 not match received CRC8, for AHT2x only*/:
      return ((reinterpret_cast<const __FlashStringHelper *>(("computed CRC8 not match received CRC8, this feature supported only by AHT2x sensors"))));
      break;

    default:
      return ((reinterpret_cast<const __FlashStringHelper *>(("unknown status"))));
      break;
  }
}

/**

 * Scans the I2C bus for devices and debugs them in the serial output

 * 

 * @param

 * @return

 */
# 510 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
void I2C_Scanner() {
  Serial.println();
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i); // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0) { // Receive 0 = success (ACK response)
      Serial.print("Found address: ");
      Serial.print(i, 10);
      Serial.print(" (0x");
      Serial.print(i, 16); // PCF8574 7 bit address
      Serial.println(")");
      count++;
    }
  }
  Serial.print("Found ");
  Serial.print(count, 10); // numbers of devices
  Serial.println(" device(s).");
}
