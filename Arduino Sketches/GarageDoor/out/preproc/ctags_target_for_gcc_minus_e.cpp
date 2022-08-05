# 1 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino"



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
# 31 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino"
// #define CHILD_ID_G_DOOR_3 2
// #define CHILD_ID_G_DOOR_4 3





// #define G_DOOR_3_PIN 32
// #define G_DOOR_4_PIN 33



# 44 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino" 2
# 45 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino" 2
# 46 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino" 2
# 47 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino" 2

// Initialize AHT10 sensor
AHTxx aht10(0x38 /*AHT15/AHT20/AHT21/AHT25 I2C address, AHT10 I2C address if address pin to GND*/, AHT1x_SENSOR);

MyMessage reportHum(4, V_HUM);
MyMessage reportTemp(5, V_TEMP);

void presentation() {
  sendSketchInfo("Garage Door", "1.0.0");

  present(4, S_HUM, "车库湿度");
  present(5, S_TEMP, "车库温度");

  present(0, S_BINARY, "车库门1");
  present(1, S_BINARY, "车库门2");
  // present(CHILD_ID_G_DOOR_3, S_BINARY);
  // present(CHILD_ID_G_DOOR_4, S_BINARY);

  // getControllerConfig().time()

  // metric = getControllerConfig().isMetric;
}

void toggle() {
  digitalWrite(25, 0x1);
  delay(50);
  digitalWrite(25, 0x0);
  delay(500);
  digitalWrite(25, 0x1);
}

void setup() {
  Wire.begin();

  I2C_Scanner();

  pinMode(25, 0x03);
  pinMode(26, 0x03);
  digitalWrite(25, 0x1);
  digitalWrite(26, 0x1);
  // pinMode(G_DOOR_3_PIN, OUTPUT);
  // pinMode(G_DOOR_4_PIN, OUTPUT);

  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Beginning to initialize AHT10 sensor")))));
  while(aht10.begin() != true) {
    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT1x not connected or failed to load calibration coefficient")))));

    delay(5000);
  }
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("AHT10 OK")))));

  toggle();

  delay(10000); // pause for 10 seconds
}

SimpleTimer AHTxx_Timer(10000 /* Temperature and humidity reading interval*/);

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
# 133 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino"
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
# 157 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino"
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
# 191 "c:\\Users\\jwwan\\Documents\\homeauto\\GarageDoor\\GarageDoor.ino"
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
