# 1 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
/**

 * @file GarageDoor.ino

 * @author Jimmy Wang

 * @brief Garage door controller

 * @version 0.2

 * @date 2022-08-08

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

SimpleTimer timer;

void reportAHTxx();
String getAHTxxStatus();
void I2C_Scanner();

// Initialize AHT10 sensor
AHTxx aht10(0x38 /*AHT15/AHT20/AHT21/AHT25 I2C address, AHT10 I2C address if address pin to GND*/, AHT1x_SENSOR);

MyMessage reportHum(4, V_HUM);
MyMessage reportTemp(5, V_TEMP);

const int ATHxxTimerId = timer.setInterval(10000 /* Temperature and humidity sample rate*/, getAHTxxStatus);

class GarageDoor {
  public:
    int statusID; // MySensors CHILDID
    int textID;
    int outputPin; // Digital pin on MCU
    int inputPin;

    // initializer
    GarageDoor(int statusID, int textID, int outputPin, int inputPin) {
      this->statusID = statusID;
      this->textID = textID;
      this->outputPin = outputPin;
      this->inputPin = inputPin;

      reportGarageDoorSwitch = new MyMessage(statusID, V_STATUS);
      reportGarageDoorState = new MyMessage(textID, V_TEXT);
    }

    bool open();
    bool close();

    bool updateState(int newState);
    void updateSensorState();

    // ----------- Setter Methods -----------

    bool setSensorState(bool newState) { return sensorState = newState; }

  private:
    char *garageDoorState[6] = {
      "关闭",
      "打开",
      "关闭中",
      "打开中",
      "静止",
    };

    bool sensorState;
    int curState, prevState;

    MyMessage *reportGarageDoorSwitch;
    MyMessage *reportGarageDoorState;

    void toggle(int it = 1);
};

bool GarageDoor::open() {
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

bool GarageDoor::updateState(int newState) {
  bool res = newState == 0 ? close() : open();

}

void GarageDoor::updateSensorState() {
  long int currentTime = millis();
}

void GarageDoor::toggle(int it) {
  for(; it > 0; --it) {
    int t = prevState;
    prevState = curState;
    if(prevState == 0) curState = 3;
    else if(prevState == 1) curState = 2;
    else if(prevState == 2) curState = 3;
    else if(prevState == 3) curState = 4;
    else if(prevState == 4) curState = 2;

    digitalWrite(outputPin, 0x1);
    delay(50);
    digitalWrite(outputPin, 0x0);
    delay(400);
    digitalWrite(outputPin, 0x1);
  }
}

GarageDoor *garageDoors[3] = {
  new GarageDoor(0, 1, 25, 36),
  
# 190 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino" 3 4
 __null
# 190 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
     ,
  new GarageDoor(2, 3, 26, 39)
};

GarageDoor *garageDoor1 = garageDoors[0], *garageDoor2 = garageDoors[2];

void __attribute__((section(".iram1" "." "28"))) handleGarageDoorInterupt1() {
  garageDoor1->updateSensorState();
}

void __attribute__((section(".iram1" "." "29"))) handleGarageDoorInterupt2() {
  garageDoor2->updateSensorState();
}

void presentation() {
  sendSketchInfo("Garage Door", "0.2");

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

  pinMode(25, 0x03);
  pinMode(26, 0x03);
  pinMode(32, 0x03);
  pinMode(33, 0x03);
  pinMode(36, 0x09);
  pinMode(39, 0x09);

  attachInterrupt(36, handleGarageDoorInterupt1, 0x03);
  attachInterrupt(39, handleGarageDoorInterupt2, 0x03);

  // Set all garage remote output pins to OFF
  digitalWrite(25, 0x1);
  digitalWrite(26, 0x1);
  digitalWrite(32, 0x1);
  digitalWrite(33, 0x1);

  garageDoor1->setSensorState(digitalRead(36));
  garageDoor2->setSensorState(digitalRead(39));

  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Beginning to initialize AHT10 sensor")))));
  while(aht10.begin() != true) {
    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT1x not connected or failed to load calibration coefficient")))));
    delay(5000);
  }
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT10 OK")))));

  delay(5000); // pause for 5 seconds
}

void loop() {

}

void receive(const MyMessage &msg) {
  // message is coming from the gateway, safety checks, and error "handling"
  if(msg.sender == 0) {
    switch(msg.type) {
      case V_STATUS:
        if(msg.sensor == 0 || msg.sensor == 2) {
          try {
            garageDoors[msg.sensor]->updateState((int)msg.data);
          } catch (int e) { Serial.println("An error occured\n" + e)}
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
# 278 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 302 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
# 336 "c:\\Users\\jwwan\\Documents\\homeauto\\Arduino Sketches\\GarageDoor\\GarageDoor.ino"
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
