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


    }

    private:
      char *garageDoorState[6] = { 
      "关闭",   // 0
      "打开",   // 1
      "关闭中", // 2
      "打开中", // 3
      "静止",   // 4
    };

    int sensorState;
    int curState, prevState;

    MyMessage *reportGarageDoorSwitch;
    MyMessage *reportGarageDoorState;

    int garageTimerId;

    void toggle(int it = 1);
}

void GarageDoor::handleTimeout() {

}

bool GarageDoor::isClosed() {
  this->readSensorState();

  if(sensorState == 1) return true;
  else return false;
}

bool GarageDoor::readSensorState() {
  int ATM = digitalRead(inputPin);

  if(ATM == this->sensorState && this->sensorState != NULL) return false;
  
  sensorState = ATM;
  return true;
}

/**
 * @brief This function emulates a button press on the actual garage door remote
 * 
 * @param it the number of times to press
 */
void GarageDoor::toggle(int it) {
  for(int i = 0; i < it; ++i) {
    if(curState == 0) this->updateState(3);
    else if(curState == 1) this->updateState(2);
    else if(curState == 2) this->updateState(3);
    else if(curState == 3) this->updateState(4);
    else if(curState == 4) this->updateState(2);

    // first time toggle no need to pause
    if(i != 0) delay(750);
    digitalWrite(outputPin, HIGH);
    delay(50);
    digitalWrite(outputPin, LOW);
    delay(400);
    digitalWrite(outputPin, HIGH);
  }
}