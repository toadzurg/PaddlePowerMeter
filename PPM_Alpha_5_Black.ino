//BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Wire.h> //For I2C (qwiic bus)
#include <Preferences.h> //For saving calibration data to EEPROM
#include "SparkFunLSM6DSO.h" //For IMU sensors
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" //For scale
#include <CircularBuffer.h> //For watts averaging

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> //For battery level
SFE_MAX1704X lipo(MAX1704X_MAX17048); // Allow access to all the 17048 features

Preferences scalePref; //Object scalePref of class Preferences
NAU7802 myScale; //Object myScale of class NAU7802
LSM6DSO myIMU; //Object myIMU of class NAU7802
CircularBuffer<int, 30> pBuffer; //Object buffer of class CircularBuffer

bool settingsDetected = false; //Used to prompt user to calibrate their scale
int imuData; //Gyroscope, accelerometer, temperature sensors
float cAngularVelocity; //Current unfiltered angular velocity X
float pAngularVelocity; //Previous unfiltered angular velocity X
float cAcclData; //Current unfiltered accelrometer data X
float pAcclData; //Previous unfiltered accelrometer data X
float cPaddleAngle; //Current fildered angle of paddle
float pPaddleAngle; //Previous filtered angle of paddle
float currentWeight; //Current load on cell
unsigned short numStrokes; //Total number of strokes
unsigned short timestamp; //Event timestamp
byte balance; //Power balance
int strokePower; //Use more descriptive language
int spm; //Stroks per minute
short watt; //Watt value to send to BLE
int xGyro, yGyro, zGyro; //Raw gyroscope data
int xAccl, yAccl, zAccl; //Raw accelrometer data
uint8_t batteryLevel; //Battery level

#define ppmService_UUID        "1818" //Cycling Power Service
#define ppmMeasurement_UUID    "2A63" //Cycling Power Measurement
#define ppmLocation_UUID       "2A5D" //Cycling Power Sensor Location
#define ppmFeature_UUID        "2A65" //Cycling Power Feature

//#define ppmControlPoint_UUID   "2A66" //Cycling Power Control Point Service

#define ppmDeviceInformation_UUID   "180A" //Device Information Service
#define ppmManufacturer_UUID   "2A29" //Manufacturer Name String
#define ppmModel_UUID          "2A24" //Model Number String 
#define ppmSN_UUID             "2A25" //Serial Number String
#define ppmSW_UUID             "2A28" //Software Revision String

#define ppmBatteryService_UUID        "180F" //Battery Service
#define ppmBatteryLevel_UUID        "2A19" //Battery Service

bool deviceConnected = false; //Used to keep track if bluetooth is connected

//PPM Server
BLEServer *ppmServer; //BLE Server

//Cycling Power Characteristics
BLECharacteristic *ppmMeasurement; //Measurements
BLECharacteristic *ppmLocation; //Device location
BLECharacteristic *ppmFeature; //Features

//Cycling Power Control Point Characteristics
//BLECharacteristic *ppmControlPoint; //Control Point

//Device Information Characteristics
BLECharacteristic *ppmManufacturer; //Manufacturer
BLECharacteristic *ppmModel; //Model
BLECharacteristic *ppmSN; //Serial number
BLECharacteristic *ppmSW; //Software version

//Battery Characteristics
BLECharacteristic *ppmBatteryLevel; //Battery level

unsigned char bleBuffer[9]; //Data buffer
unsigned short flags = 0x23; //BLE CPS flags 

void sendPower(){
  //If client is connected to power meter
  if(deviceConnected){
    //Set data in buffers
    bleBuffer[0] = flags & 0xff;
    bleBuffer[1] = (flags >> 8) & 0xff;
    bleBuffer[2] = watt & 0xff;
    bleBuffer[3] = (watt >> 8) & 0xff;
    bleBuffer[4] = balance & 0xff;
    bleBuffer[5] = numStrokes & 0xff;
    bleBuffer[6] = (numStrokes >> 8) & 0xff;
    bleBuffer[7] = timestamp & 0xff;
    bleBuffer[8] = (timestamp >> 8) & 0xff;

    //Set value and notify
    ppmMeasurement->setValue((unsigned char *)&bleBuffer, 9);
    ppmMeasurement->notify();

    //For troubleshooting
    /*Serial.print("flags ");
    Serial.println(flags);
    Serial.print("watt ");
    Serial.println(watt);
    Serial.print("balance ");
    Serial.println(balance);
    Serial.print("numStrokes ");
    Serial.println(numStrokes);
    Serial.print("timestamp ");
    Serial.println(timestamp);
    Serial.print("SPM ");
    Serial.println(spm);
    Serial.println("");*/
  }
}

//Reset BLE connection
void resetConnection(){
  Serial.println("Resetting Paddle Power Meter...");
  
  startAdvertising(); //Start advertising BLE
}

//listen for BLE connect/disconnect events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* ppmServer) {
      deviceConnected = true;
      Serial.println("Connected to server");
    };

    void onDisconnect(BLEServer* ppmServer){
      deviceConnected = false;
      Serial.println("Disconnected from server");
      resetConnection();
    }
};

//Listen for client input to Cycling Power Control Point Characteristic
/*class MyControlPointCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ppmControlPoint) {
      std::string value = ppmControlPoint->getValue();
      if (value.length() > 0){
        Serial.println("*********");
        Serial.print("Value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    };
};*/

void setup(){
  Serial.begin(115200); //Set baud rate
  Serial.println("Starting setup...");

  Wire.begin(); //Start I2C (Qwiic) bus
  Wire.setClock(400000); //Qwiic Scale is capable of running at 400kHz if desired
  scalePref.begin("scale", false); //For writing calibration data to EEPROM... not sure if this should be moved?
  //scalePref.end();// not sure if this is needed?
  delay(500); //Delay half a second to let everything catch up

   //Check battery is working
  if(lipo.begin()){
    Serial.println("Battery ready");    
  }
  else{
    Serial.println(F("Fuel gauge not detected. Please check wiring. Freezing."));
    while (1);
  }

  //Check inertial measurement unit is working
  if(myIMU.begin()){
    Serial.println("IMU ready");
  }
  else{
    Serial.println("Could not connect to IMU. Check wiring and reboot...");
    while (1);
  }
  //Load IMU basic settings
  if(myIMU.initialize(BASIC_SETTINGS)){
    Serial.println("Loaded IMU basic settings");
  }
  else{
    Serial.println("Could not load IMU basic settings... reboot");
    while(1);
  }
  //Check weight scale is working
  if (myScale.begin()){
    Serial.println("Weight scale ready");
    scaleSetup(); //Setup scale
    Serial.println("Done setting up scale");
  }
  else{
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  startServices(); //Start setting up BLE stuff
}

//Set up BLE server and services
void startServices(){
  BLEDevice::init("Paddle Power Meter"); //Server name
  ppmServer = BLEDevice::createServer(); //Create server
  ppmServer->setCallbacks(new MyServerCallbacks()); //Listen for connect/disconnect events

  //BLE Services
  BLEService *ppmService = ppmServer->createService(ppmService_UUID);
  BLEService *ppmDeviceInfo = ppmServer->createService(ppmDeviceInformation_UUID);
  BLEService *ppmBattery = ppmServer->createService(ppmBatteryService_UUID);
  //BLEService *ppmCtrlPoint = ppmServer->createService(ppmControlPoint_UUID);


  //BLE cycling power Characteristics
  ppmMeasurement = ppmService->createCharacteristic(ppmMeasurement_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  ppmLocation = ppmService->createCharacteristic(ppmLocation_UUID, BLECharacteristic::PROPERTY_READ);
  ppmFeature = ppmService->createCharacteristic(ppmFeature_UUID, BLECharacteristic::PROPERTY_READ);

  //BLE battery Characteristics
  ppmBatteryLevel = ppmBattery->createCharacteristic(ppmBatteryLevel_UUID, BLECharacteristic::PROPERTY_READ);
  
  //BLE cycling power control point Characteristics
  //ppmControlPoint = ppmService->createCharacteristic(ppmControlPoint_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  //BLE device info Characteristics
  ppmManufacturer = ppmDeviceInfo->createCharacteristic(ppmManufacturer_UUID, BLECharacteristic::PROPERTY_READ);
  ppmModel = ppmDeviceInfo->createCharacteristic(ppmModel_UUID, BLECharacteristic::PROPERTY_READ);
  ppmSN = ppmDeviceInfo->createCharacteristic(ppmSN_UUID, BLECharacteristic::PROPERTY_READ);
  ppmSW = ppmDeviceInfo->createCharacteristic(ppmSW_UUID, BLECharacteristic::PROPERTY_READ);

  //BLE Descriptors
  ppmMeasurement->addDescriptor(new BLE2902());
  ppmBatteryLevel->addDescriptor(new BLE2902());
  //ppmControlPoint->addDescriptor(new BLE2902());

  //Set callbacks... use this to listen for user input like calibration prompt, pause, resume, ect
  //ppmControlPoint->setCallbacks(new MyControlPointCallbacks());

  //Start BLE services
  ppmService->start();
  ppmDeviceInfo->start();
  ppmBattery->start();
  //ppmCtrlPoint->start();

  //Power meter location
  byte posvalue = 5; //Left crank
  ppmLocation->setValue((uint8_t *)&posvalue, 1);

  //Cycling Power Feature flags
  unsigned char fBuffer[4];
  fBuffer[0] = 0x01; //Crank length adjustment supported - 0001
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x09; //Pedal power balance and crank revolution data supported - 1001
  ppmFeature->setValue((unsigned char *)&fBuffer, 4);

  //unsigned char cpBuffer[1];
  //cpBuffer[0] = 0x04;
  //ppmControlPoint->setValue((unsigned char *)&cpBuffer, 1);

  //Device info
  String manufacturerName = "Benjie PPM"; //Manufacturer Name
  String model = "Alpha 1.0"; //Model 
  String snNum = "1"; //Serial Number
  String swNum = "1.0"; //Software Version
  ppmManufacturer->setValue((uint8_t *)&manufacturerName, 10);
  ppmModel->setValue((uint8_t *)&model, 5);
  ppmSN->setValue((uint8_t *)&snNum, 10);
  ppmSW->setValue((uint8_t *)&swNum, 10);

  //Set battery level
  uint8_t batteryLevel = lipo.getSOC();
  ppmBatteryLevel->setValue((uint8_t *)&batteryLevel, 1);
  ppmBatteryLevel->notify();

  startAdvertising();//Start advertising ppm
}

//Advertise Paddle Power Meter
void startAdvertising(){  
  BLEAdvertising *ppmAdvertising = BLEDevice::getAdvertising();
  ppmAdvertising->addServiceUUID(ppmService_UUID);
  ppmAdvertising->setScanResponse(true);
  ppmAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  ppmAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Paddle Power Meter Ready!");
}

//Set up scale
void scaleSetup(){
  readSystemSettings(); //Load zeroOffset and calibrationFactor from EEPROM

  myScale.setSampleRate(NAU7802_SPS_320); //Increase to max sample rate
  myScale.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel 

  Serial.print("Scale Zero offset: ");
  Serial.println(myScale.getZeroOffset());
  Serial.print("Scale calibration factor: ");
  Serial.println(myScale.getCalibrationFactor());
}

//Calibrate scale --rewrite to use BLE
void calibrateScale(void){
  Serial.println();
  Serial.println(F("Scale calibration"));

  Serial.println(F("Setup scale with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  myScale.calculateZeroOffset(64); //Zero or Tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(myScale.getZeroOffset());

  Serial.println(F("Place known weight on scale. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  Serial.println(F("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): "));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  //Read user input
  float weightOnScale = Serial.parseFloat();
  Serial.println();

  myScale.calculateCalibrationFactor(weightOnScale, 64); //Tell the library how much weight is currently on it
  Serial.print(F("New cal factor: "));
  Serial.println(myScale.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(myScale.getWeight(), 2);

  recordSystemSettings(); //Commit these values to EEPROM
}

//Record the current settings to EEPROM
void recordSystemSettings(void){
  //Get various values from the library and commit them to NVM
  scalePref.putFloat("calFac", myScale.getCalibrationFactor());
  scalePref.putLong("zOff", myScale.getZeroOffset());
}

//Reads the current system settings from EEPROM
//If anything looks weird, reset setting to default value
void readSystemSettings(void){
  float settingCalibrationFactor; //Value used to convert the load cell reading to lbs or kg
  long settingZeroOffset; //Zero value that is found when scale is tared

  settingCalibrationFactor = scalePref.getFloat("calFac", 0);
  //Look up the calibration factor
  if (settingCalibrationFactor == 0xFFFFFFFF){
    settingCalibrationFactor = 0; //Default to 0
    scalePref.putFloat("calFac", myScale.getCalibrationFactor());
  }

  //Look up the zero tare point
  settingZeroOffset = scalePref.getLong("zOff", 0);
  if (settingZeroOffset == 0xFFFFFFFF){
    settingZeroOffset = 1000L; //Default to 1000 so we don't get inf
    scalePref.putLong("zOff", myScale.getZeroOffset());
  }

  //Pass these values to the library
  myScale.setCalibrationFactor(settingCalibrationFactor);
  myScale.setZeroOffset(settingZeroOffset);

  settingsDetected = true; //Assume for the moment that there are good cal values
  if (settingCalibrationFactor < 0.1 || settingZeroOffset == 1000){
    settingsDetected = false; //Defaults detected. Prompt user to cal scale.
  }
}

uint32_t lmillis = 0; //Timer
//Main loop
void loop(){
  //Read sensor data every .1 seconds
  if ((millis() - lmillis > 100) && deviceConnected){
    lipoStuff(); //Battery data
    scaleStuff(); //Scale data
    imuStuff(); //Accelerometer/gyroscope data
    lmillis = millis(); //Previous time = current time
  }
}

uint32_t powerMillis = 0; //Timer

void lipoStuff(void){
  if (millis() - powerMillis > 60000){
    Serial.print("Voltage: ");
    Serial.print(lipo.getVoltage());  // Print the battery voltage
    Serial.println("V");

    Serial.print("Battery Percentage: ");
    batteryLevel = lipo.getSOC();
    Serial.print(batteryLevel); // Print the battery state of charge with 2 decimal places
    Serial.println("%");

    ppmBatteryLevel->setValue((uint8_t *)&batteryLevel, 1);
    ppmBatteryLevel->notify();
    powerMillis = millis();

    Serial.println(" ");
  }
}

//Read load on scale and listen for calibration prompt -- rewrite this to use BLE input (Cycling Power Service, Control Point Characteristic)
void scaleStuff(void){
  if (myScale.available()){
    long currentReading = myScale.getReading();
    currentWeight = myScale.getWeight();

    if(settingsDetected == false){
      Serial.println("Scale not calibrated. Enter 'c'.");
      if (Serial.available()){
        while (Serial.available()) Serial.read(); //Clear RX buffer
        while (Serial.available() == 0) delay(10); //Wait for user to press key
      }
    }

    if (Serial.available()){
      byte incoming = Serial.read();
      if (incoming == 't'){ //Tare the scale
        Serial.println("Taring Scale");
        myScale.calculateZeroOffset();
        Serial.print(F("New zero offset: "));
        Serial.println(myScale.getZeroOffset());
      } 
      else if (incoming == 'c'){ //Calibrate
        Serial.println("Calibrating Scale");
        calibrateScale();
      }
    }
  }
}

//For calculating elapsed time in order to find # of strokes and stroke rate, and timers for BLE transmission
bool strokeFlag = true;
uint32_t strokeMillis;
float counter = 0;
double i_prev = 0;
double i_curr = 0;
double i_diff = 0;
int i_time1 = 0;
int i_time2 = 0;

void imuStuff(void){
  imuData = myIMU.listenDataReady();
  //If IMU is ready and BLE are connected
  if (imuData == ALL_DATA_READY){
    //Accelerometer/gyro data
    xGyro = myIMU.readFloatGyroX(); //Current raw angular velocity of Y
    //yGyro = myIMU.readFloatGyroY(); //Current raw angular velocity of X
    //zGyro = myIMU.readFloatGyroZ(); //Current raw angular velocity of Z
    //xAccl = myIMU.readFloatAccelX(); //Current raw acceleration of X
    //yAccl = myIMU.readFloatAccelY(); //Current raw acceleration of Y
    //zAccl = myIMU.readFloatAccelZ(); //Current raw acceleration of Z

    cAngularVelocity = xGyro;
    strokePower = ((cAngularVelocity / 360 ) * 11 * currentWeight); //Calculate instantaneous power (Angular Velocity / 360 * Radius * Load)
    spm = ((cAngularVelocity / 360) * 60); //Strokes per minute using gyro

    counter = counter + 1; //Increment counter
    //calculate number of strokes 
    if((strokePower > 0) && (spm > 0)){ //Only look at positive power and stroke rates
      pBuffer.push(strokePower); //Add power value to circular buffer
      balance = 99; //testing L/R power balance 
      if ((strokePower > 5) && (strokeFlag) && (millis() - strokeMillis > 500)){ //Only count the stroke if power is > 5, if stroke flag is true, and skip stroke if last stroke is less than .5 seconds ago (max 120 spm)
        strokeFlag = false; //Set to false
        numStrokes = numStrokes + 1; //Increment stroke count
        i_curr = counter; //Find difference between previous and current stroke
        i_diff = i_curr - i_prev;
        timestamp = timestamp + (unsigned short)(i_diff*(1024/8)); //Stroke time stamp (1/1024 second format is required by BLE...) Time = previous time + (difference *(1024/approximate sample rate per second) How to calculate real sample rate?
        i_prev = i_curr;
        strokeMillis = millis();
      }

      //Calculate continuous ~3 second average power
      //Send readings update every ~second
      if (i_time2 == 10){
        float avg = 0.0;
        using index_t = decltype(pBuffer)::index_t;
        for (index_t i = 0; i < pBuffer.size(); i++) {
          avg += pBuffer[i] / (float)pBuffer.size();
        }
        watt = avg;
        sendPower(); //Send the money off to BLE!
        i_time2 = 0; //Reset timers
        i_time1 = 0;
      }
      i_time2 ++;
    }
    else{
      strokeFlag = true; //Set stroke flag to true
      //Clear out power buffer when idle for ~3 seconds
      if (i_time1 == 30){
        pBuffer.clear(); //Clear out the power buffer
        watt = 0; //Set to 0 power
        sendPower(); //Send it off
        i_time1 = 0; //Reset timers
        i_time2 = 0;
      }
      i_time1 ++;
    }
  }
}

// Start of code to calculate l/r power balance (how to figure out with side is left or right?) and orientation in space to calculate stroke efficiency
/*
#include <Eigen.h> // Include Eigen library for matrix operations

// Kalman filter variables
Eigen::VectorXf state(4); // State [x, x_dot, y, y_dot]
Eigen::MatrixXf P(4, 4); // State covariance matrix
Eigen::MatrixXf Q(4, 4); // Process noise covariance matrix
Eigen::MatrixXf R(2, 2); // Measurement noise covariance matrix
Eigen::MatrixXf H(2, 4); // Measurement matrix
Eigen::MatrixXf I(4, 4); // Identity matrix

void setupKalmanFilter() {
  // Initialize Kalman filter parameters
  P.setIdentity();
  Q.setIdentity();
  R.setIdentity();
  H.setIdentity();
  I.setIdentity();
}

void updateKalmanFilter(float ax, float ay, float gx, float gy) {
  // Kalman filter prediction step
  state = state; // Update state with your model here
  P = P + Q;

  // Kalman filter update step
  Eigen::VectorXf measurement(2);
  measurement << ax, ay; // Replace with actual accelerometer readings
  Eigen::VectorXf y = measurement - H * state;
  Eigen::MatrixXf S = H * P * H.transpose() + R;
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  state = state + K * y;
  P = (I - K * H) * P;
}

void imuStuff(void) {
  // ... existing code ...

  // Read accelerometer data
  float xAccl = myIMU.readFloatAccelX();
  float yAccl = myIMU.readFloatAccelY();
  float zAccl = myIMU.readFloatAccelZ();

  // Complementary Filter
  // Assuming 0.98 and 0.02 as filter coefficients and dt as the time difference
  float angle = 0.98 * (angle + xGyro * dt) + 0.02 * xAccl;

  // Update Kalman Filter
  updateKalmanFilter(xAccl, yAccl, xGyro, yGyro);

  // Use angle from Complementary Filter or state from Kalman Filter for orientation

  // ... remaining code ...
}
*/

