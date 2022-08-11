# 1 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 40 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
// MySensors child IDs




// #define CHILD_ID_G_DOOR_3 2
// #define CHILD_ID_G_DOOR_4 3



// Hardware input & output pins
# 65 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
# 66 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 67 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 68 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2
# 69 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 2

void handleGarageDoorCallback();
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

    // initializer
    GarageDoor(int statusID, int textID, int outputPin, int inputPin) {
      this->statusID = statusID;
      this->textID = textID;
      this->outputPin = outputPin;
      this->inputPin = inputPin;

      sensorState = 
# 99 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
                   __null
# 99 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
                       ;
      prevState = 
# 100 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
                 __null
# 100 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
                     ;
      curState = 
# 101 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
                __null
# 101 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
                    ;

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
  garageTimerId = timer.setTimeout(12*1000 /* 12s to fully open*/, handleGarageDoorCallback);
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
  garageTimerId = timer.setTimeout(15*1000 /* 15s to fully close*/, handleGarageDoorCallback);
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

  if(ATM == sensorState && sensorState != 
# 258 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
                                         __null
# 258 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
                                             ) return false;

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

    digitalWrite(outputPin, 0x1);
    delay(50);
    digitalWrite(outputPin, 0x0);
    delay(400);
    digitalWrite(outputPin, 0x1);
  }
}

GarageDoor *garageDoors[3] = {
  new GarageDoor(0, 1, 25, 13),
  
# 282 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
 __null
# 282 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
     ,
  new GarageDoor(2, 3, 26, 14)
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
  sendSketchInfo("Garage Door", "0.3");

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
  timer.enable(AHTxxTimerID);

  timer.setInterval(100, handleCheckSensorCallback);
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
# 375 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 399 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 433 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
