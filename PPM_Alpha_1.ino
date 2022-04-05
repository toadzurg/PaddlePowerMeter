//BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Wire.h> //For I2C (qwiic bus)
#include <Preferences.h> //For saving calibration data to EEPROM
#include "SparkFunLSM6DSO.h" //For IMU sensors
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" //For scale
#include <SparkFun_RV8803.h> //For real time clock

Preferences scalePref; //Object scalePref of class Preferences
NAU7802 myScale; //Object myScale of class NAU7802
LSM6DSO myIMU; //Object myIMU of class NAU7802
RV8803 rtc; //Object etc of class NAU7802

bool settingsDetected = false; //Used to prompt user to calibrate their scale
int imuData; //Gyroscope, accelerometer, temperature sensors
float cAngularVelocity; //Current filtered angular velocity X
float pAngularVelocity; //Previous filtered angular velocity X
float cAcclData; //Current filtered accelrometer data X
float currentWeight; //Current load on cell
float paddleAngle; //Current angle of paddle
unsigned short numStrokes = 0; //Total number of strokes
unsigned short timestamp = 0; //Event timestamp
String currentDate, currentTime; //RTC values
byte currentSecond, currentHundredth; //RTC values
int time1, time2, time3, time4, time5; //Use more descriptive language
int power, power1, power2, power3, power4, power5; //Use more descriptive language
float spm; //Stroks per minute
short powerReading = 0; //Power reading to send to BLE
int rawGyroData; //Raw gyroscope data
float LFP_Beta = 0.5; //Gyro filter weight

#define ppmService_UUID        "1818" //Cycling Power Service
#define ppmMeasurement_UUID    "2A63" //Cycling Power Measurement
#define ppmLocation_UUID       "2A5D" //Cycling Power Sensor Location
#define ppmFeature_UUID        "2A65" //Cycling Power Feature
//#define ppmControlPoint_UUID   "2A66" //Cycling Power Control Point

#define ppmDeviceInformation_UUID   "180A" //Device Information Service
#define ppmManufacturer_UUID   "2A29" //Manufacturer Name String
#define ppmModel_UUID          "2A24" //Model Number String 
#define ppmSN_UUID             "2A25" //Serial Number String
#define ppmSW_UUID             "2A28" //Software Revision String

bool deviceConnected = false; //Used to keep track if bluetooth is connected

BLEServer *ppmServer; //BLE Server
//Cycling Power Characteristics
BLECharacteristic *ppmMeasurement; //
BLECharacteristic *ppmLocation; //
BLECharacteristic *ppmFeature; //
//BLECharacteristic *ppmControlPoint; //
BLECharacteristic *ppmManufacturer; //
BLECharacteristic *ppmModel; //
BLECharacteristic *ppmSN; //
BLECharacteristic *ppmSW; //

unsigned char bleBuffer[8]; //Data buffer
unsigned short flags = 0x20; //BLE CPC flags

void sendPower(){
  //If client is connected to power meter
  if(deviceConnected){
    //Set data in buffers
    bleBuffer[0] = flags & 0xff;
    bleBuffer[1] = (flags >> 8) & 0xff;
    bleBuffer[2] = powerReading & 0xff;
    bleBuffer[3] = (powerReading >> 8) & 0xff;
    bleBuffer[4] = numStrokes & 0xff;
    bleBuffer[5] = (numStrokes >> 8) & 0xff;
    bleBuffer[6] = timestamp & 0xff;
    bleBuffer[7] = (timestamp >> 8) & 0xff;

    //Set value and notify
    ppmMeasurement->setValue((unsigned char *)&bleBuffer, 8);
    ppmMeasurement->notify();

    //For troubleshooting
    Serial.print("flags ");
    Serial.println(flags);
    Serial.print("powerReading ");
    Serial.println(powerReading);
    Serial.print("numStrokes ");
    Serial.println(numStrokes);
    Serial.print("timestamp ");
    Serial.println(timestamp);
    Serial.println("");
    Serial.print("SPM ");
    Serial.println(spm);
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

//Listen for user input
class cpsCallBack: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ppmCharacteristic) {
      std::string value = ppmCharacteristic->getValue();
      if (value.length() > 0){
        Serial.println("*********");
        Serial.print("Value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    };
};

void setup(){
  Serial.begin(115200); //Set baud rate
  Serial.println("Starting setup...");

  Wire.begin(); //Start I2C (Qwiic) bus
  Wire.setClock(400000); //Qwiic Scale is capable of running at 400kHz if desired
  scalePref.begin("scale", false); //For writing calibration data to EEPROM... not sure if this should be moved?
  //scalePref.end();// not sure if this is needed?
  delay(500); //Delay half a second to let everything catch up
  
  //Check real time clock is working
  if (rtc.begin()){
    if (rtc.setToCompilerTime()){ //This is 2.5 hours off for some reason... look into later
      Serial.println("New time set!");
    }
    else{
      Serial.println("Something went wrong setting the time");
    }
    Serial.println("RTC ready");
  }
  else{
    Serial.println("Something went wrong, check wiring");
    while(1);
  }
  //Check inertial measurement unit is working
  if(myIMU.begin()){
    Serial.println("IMU ready");
  }
  else {
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

  //BLE Characteristics
  ppmMeasurement = ppmService->createCharacteristic(ppmMeasurement_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  ppmLocation = ppmService->createCharacteristic(ppmLocation_UUID, BLECharacteristic::PROPERTY_READ);
  ppmFeature = ppmService->createCharacteristic(ppmFeature_UUID, BLECharacteristic::PROPERTY_READ);
  //ppmControlPoint = ppmService->createCharacteristic(ppmControlPoint_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);

  ppmManufacturer = ppmDeviceInfo->createCharacteristic(ppmManufacturer_UUID, BLECharacteristic::PROPERTY_READ);
  ppmModel = ppmDeviceInfo->createCharacteristic(ppmModel_UUID, BLECharacteristic::PROPERTY_READ);
  ppmSN = ppmDeviceInfo->createCharacteristic(ppmSN_UUID, BLECharacteristic::PROPERTY_READ);
  ppmSW = ppmDeviceInfo->createCharacteristic(ppmSW_UUID, BLECharacteristic::PROPERTY_READ);

  //BLE Descriptor
  ppmMeasurement->addDescriptor(new BLE2902());

  //Set callbacks... use this to listen for user input like calibration prompt, pause, resume, ect
  //cpsCallBack->setCallbacks(new cscMeasurementCallbacks());

  //Start BLE services
  ppmService->start();
  ppmDeviceInfo->start();

  //Power meter location
  byte posvalue = 0; //Other
  ppmLocation->setValue((uint8_t *)&posvalue, 1);

  //Feature flags
  unsigned char fBuffer[4];
  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;
  ppmFeature->setValue((unsigned char *)&fBuffer, 4);

  //Device info
  String manufacturerName = "Benjie PPM"; //Manufacturer Name
  String model = "Alpha 1.0"; //Model 
  String snNum = "1"; //Serial Number
  String swNum = "1.0"; //Software Version
  ppmManufacturer->setValue((uint8_t *)&manufacturerName, 10);
  ppmModel->setValue((uint8_t *)&model, 5);
  ppmSN->setValue((uint8_t *)&snNum, 10);
  ppmSW->setValue((uint8_t *)&swNum, 10);
 
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

uint32_t pMillis = 0; //Timer
//Main loop
void loop() {
  //Read sensor data every .1 seconds
  if (millis() - pMillis > 100){
    rtcStuff(); //Clock data
    scaleStuff(); //Scale data
    imuStuff(); //Accelerometer/gyroscope data
    pMillis = millis(); //Previous time = current time
  }
}

//Real time clock data, not using for now...
void rtcStuff(void){
  if (rtc.updateTime()){ //Updates the time variables from RTC
    currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format
    currentTime = rtc.stringTime(); //Get the time HH/MM/SS
    currentSecond = rtc.getSeconds(); //Get the second SS
  }
  else{
    Serial.println("RTC read failed");
  }
}

//Read load on scale and listen for calibration prompt -- rewrite this to use BLE input (Cycling Power Service, Control Point Characteristic)
void scaleStuff(void){
  if ((myScale.available() == true) && (deviceConnected)){
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
      byte incoming = Serial.read();;
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

//Needed for calculating elapsed time in order to find stroke rate and # of strokes
bool strokeFlag = true;
float counter = 0;
double i_prev = 0;
double i_curr = 0;
double i_diff = 0;

void imuStuff(void){
  imuData = myIMU.listenDataReady();

  //If IMU is ready and BLE are connected
  if ((imuData == ALL_DATA_READY) && (deviceConnected)){
    //Accelerometer stuff
    //Serial.println("X = ");
    //Serial.println(myIMU.readFloatAccelX(), 3);
    //Serial.print(" Y = ");
    //Serial.println(myIMU.readFloatAccelY(), 3);
    //Serial.print("Z = ");
    //Serial.println(myIMU.readFloatAccelZ(), 3);

    rawGyroData = (myIMU.readFloatGyroX()); //Current raw angular velocity of X
    cAngularVelocity = rawGyroData; // cAngularVelocity - LFP_Beta * (cAngularVelocity - rawGyroData);

    //Somehow calculate R radius.... look into using accelerometer and integrate data / time to calculate angle. Will need a good filter through, look into implementing kalman filter 
    power = ((cAngularVelocity / 360 ) * 10 * currentWeight); //Calculate instantaneous power (Angular Velocity / 360 * Radius * Load)

    //Attempt at finding paddle angle... not needed for now, work more on it later
    //paddleAngle += (((cAngularVelocity / 360) * (millis() - pMillis)) + (cAngularVelocity / 360) ); //Calculate change in paddle angle by integrating angular velocity?
    //paddleAngle = .98 * ((paddleAngle + (cAngularVelocity / 360) * (millis() - pMillis))) + .02 * cAcclData;
    //pAngularVelocity = cAngularVelocity; //Previous angular velocity = current angular velocity

    spm = ((cAngularVelocity / 360) * 60); //Strokes per minute
    
    //calculate number of strokes + elasped time for SPM (BLE CPS has to do it this way in order to work with Garmin...)
    if((power > 0) && (spm > 0)){ //Only look at positive power and stroke rates
      counter = counter + 1; //Increment counter
      if (strokeFlag){ //Assume true for now
        strokeFlag = false; //Set to false
        numStrokes = numStrokes + 1; //Increment stroke count
        i_curr = counter-1; //Next two lines are to find time difference between this and last stroke
        i_diff = i_curr - i_prev;
        timestamp = timestamp + (unsigned short)(i_diff*(1024/5)); //Set time stamp (weird format is required by BLE...)
        i_prev = i_curr;
      }

      powerReading += power; //Cumulative power
      //Send readings update every second
      if (time2 == 10){
        powerReading = powerReading/10; //Remember we are operating at .1 tick rate 
        sendPower(); //Send the money off to BLE!
        powerReading = 0; //Reset cumulative power
        time2 = 0;
        time1 = 0;
        }
        time2 ++;
    }
    else{
      strokeFlag = true;
    }

    //Set readings to 0 when idle for 3 seconds
    if((power <= 0) && (spm <= 0)){
      if (time1 == 30){
      powerReading = 0;
      sendPower(); //Send 0 power to BLE
      time1 = 0;
      time2 = 0;
      }
      time1 ++;
    }

    // For future use
    //Serial.print("Change in angle of paddle: ");
    //Serial.println(paddleAngle);
    //Serial.print("Paddle angle left: ");
    //Serial.println(lAngle);
    //Serial.print("Paddle angle right: ");
    //Serial.println(rAngle);
    //Serial.print("Left side average power: ");
    //Serial.println(lPower);
    //Serial.print("Right side average power: ");
    //Serial.println(rPower);
  }
}