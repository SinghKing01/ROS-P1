const long referenceMv = 5000;

void setup() {
// this function runs once, when the microcontroller starts to set up the serial port
  Serial.begin(9600);
}

void loop() {
// this function runs continuously, listening the serial port
// and, as required, sampling the ADC and sending the corresponding distance

  if (Serial.available() > 0) {
    // read the incoming byte:
    int inData = Serial.read() - 48;

    if(inData == 0){
      readAnalog(A0);
    }else if(inData == 1){
      readAnalog(A1);
    }else if(inData == 2){
      readAnalog(A2);
    }
  }

}

void readAnalog(int an_port) {
// this function samples the ADC, transforms the incoming voltage in mV into
// a distance in cm and sends the latter as a well-formatted packet
// through the serial port

  int val = analogRead(an_port);
  int mV = (val * referenceMv) / 1023;

  int cm = 15;

  if (mV <= 2750) {
    cm = getDistance(mV);
  }

  // send measure through the serial port
  Serial.print("R");
  Serial.print(an_port-A0); // A0 is port 14, A1 is port #15 and A2 is port #16
  Serial.print(":");
  if (cm < 100) Serial.print(0);
  Serial.println(cm);

}

// distance interpolation, 250mV intervals
const int TABLE_ENTRIES = 12;
const int INTERVAL  = 250;
// look-up table
static int distance[TABLE_ENTRIES] = {150,140,130,100,60,50,40,35,30,25,20,15};

int getDistance(int mV) {
// this function transforms a voltage in mV from the ADC into a distance in cm
  if (mV > INTERVAL * TABLE_ENTRIES - 1)      return distance[TABLE_ENTRIES - 1];
  else {
    int index = mV / INTERVAL;
    float frac = (mV % 250) / (float)INTERVAL;
    return distance[index] - ((distance[index] - distance[index + 1]) * frac);
  }
}
