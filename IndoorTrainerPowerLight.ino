#include "BLEDevice.h"
#include <Adafruit_NeoPixel.h>

// neopixel stuff
#define PIN         4  // IO4 on esp32
#define NUMPIXELS   48 // whatever you have
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// BLE server we wish to connect to
BLEAdvertisedDevice* IndoorTrainerDevice;
BLEClient *bleClient;

#define BLE_NOT_CONNECTED   0
#define BLE_SCANNING        1
#define BLE_SCANNED         2
#define BLE_CONNECTED       3

uint8_t bleStatus = BLE_NOT_CONNECTED;

BLERemoteService *pSvcCyclingSpeedCadence; // UUID 0x1816
BLERemoteService *pSvcCyclingPower; // UUID 0x1818
BLERemoteService *pSvcFitnessMachine; // UUID 0x1826

BLERemoteCharacteristic *pCharCyclingSpeedCadenceMeasurement; // UUID 0x2A5B
BLERemoteCharacteristic *pCharCyclingPowerMeasurement; // UUID 0x2A63
BLERemoteCharacteristic *pCharIndoorBikeData; // UUID 0x2AD2
BLERemoteCharacteristic *pCharTrainingStatus; // UUID 0x2AD3
BLERemoteCharacteristic *pCharFitnessMachineStatus; // UUID 0x2ADA
BLERemoteCharacteristic *pCharFitnessMachineControlPoint; // UUID 0x2AD9

uint32_t bleCumulativeWheelRevsCSC; // zitten ook in CyclingPower characteristic
uint16_t bleLastWheelEventTimeCSC; // zitten ook in CyclingPower characteristic
uint16_t bleCumulativeCrankRevsCSC;// zitten ook in CyclingPower characteristic
uint16_t bleLastCrankEventTimeCSC; // zitten ook in CyclingPower characteristic
uint16_t bleInstantaneousPower; 
uint8_t blePedalPowerBalance;
uint32_t bleCumulativeWheelRevs;
uint16_t bleLastWheelEventTime;
uint16_t bleCumulativeCrankRevs;
uint16_t  bleLastCrankEventTime;
float     bleInstantaneousSpeed; // 16-bit field in indoor bike data / 100
uint16_t bleInstantaneousCadence;
uint32_t bleCumulativeDistance;
uint16_t bleInstantaneousPowerIBD; 
uint16_t bleElapsedTime;
uint16_t bleResistanceLevelIBD; // reports back the setting by FTMS Control Point

// using the global power & cadence to set light color & brightness
// bleInstantaneousCadence, bleInstantaneousPowerIBD

//https://www.instructables.com/How-to-Make-Proper-Rainbow-and-Random-Colors-With-/
const uint8_t HSVpower[121] = 
{0, 2, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 25, 28, 30, 32, 34, 36, 38, 40,
42, 45, 47, 49, 51, 53, 55, 57, 59, 62, 64, 66, 68, 70, 72, 74, 76, 79, 81, 
83, 85, 87, 89, 91, 93, 96, 98, 100, 102, 104, 106, 108, 110, 113, 115, 117, 
119, 121, 123, 125, 127, 130, 132, 134, 136, 138, 140, 142, 144, 147, 149, 
151, 153, 155, 157, 159, 161, 164, 166, 168, 170, 172, 174, 176, 178, 181, 
183, 185, 187, 189, 191, 193, 195, 198, 200, 202, 204, 206, 208, 210, 212, 
215, 217, 219, 221, 223, 225, 227, 229, 232, 234, 236, 238, 240, 242, 244, 
246, 249, 251, 253, 255};

// the 'power-conscious' HSV rainbow
void powerHSV(int angle, uint8_t *red, uint8_t *green, uint8_t *blue)
{
  if (angle<120) {*red = HSVpower[120-angle]; *green = HSVpower[angle]; *blue = 0;} else
  if (angle<240) {*red = 0;  *green = HSVpower[240-angle]; *blue = HSVpower[angle-120];} else
                 {*red = HSVpower[angle-240]; *green = 0; *blue = HSVpower[360-angle];}
}

static int power2angle (uint16_t power) {
  int angle;
  if (power > 400) power = 400;
  angle = 240 - power*3 / 4;
  while (angle < 0) angle += 360;
  return angle;
}
static uint8_t cadence2brightness (uint16_t cadence) {
  uint32_t brightness;
  if (cadence > 128) cadence = 128;
  brightness = 32 + (cadence*223) / 128;
  return (uint8_t) brightness;
}

static void updateLights () {
  uint8_t brightness;
  uint8_t red, green, blue;
  int angle;

  angle = power2angle (bleInstantaneousPowerIBD);
  powerHSV (angle, &red,&green,&blue);
  brightness = cadence2brightness (bleInstantaneousCadence);

  uint32_t r,g,b;
  r = (uint32_t) red; g= (uint32_t) green; b = (uint32_t) blue;
  r = r*brightness / 255;
  g = g*brightness / 255;
  b = b*brightness / 255;

  for (uint8_t i=0;i<NUMPIXELS;i++) {
    pixels.setPixelColor(i,pixels.Color(r,g,b));
  }
  pixels.show();
} // updateLights

static void callbackIndoorBikeData (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  uint16_t flags;
  uint8_t idx;
  // parse the pData according to the spec
  flags = *(uint16_t*)(pData);
  idx = 2;
  if ((flags & 0x1) == 0) {
    // C1 contains instantaneous speed
    // 16-bit field is km/h with 0.01 resolution
    bleInstantaneousSpeed =  ((float)(*(uint16_t*)(pData+idx))) / 100.0;
    idx += 2;
  }
  if (flags & 0x2){
    // C2 average speed - skip
    idx += 2;
  }
  if (flags & 0x4) {
    // C3 instantaneous cadence
    // 16-bit field is cadence with 0.5 resolution
    bleInstantaneousCadence =  (*(uint16_t*)(pData+idx)) >> 1;
    idx += 2;
  }
  if (flags & 0x8) {
    // C4 average cadence -- skip
    idx += 2;
  }
  if (flags & 0x10) {
    // C5 total distance - 3 bytes
    bleCumulativeDistance =  (*(uint32_t*)(pData+idx))&0xFFFFFF; // 24-bits field
    idx += 3;
  }
  if (flags & 0x20) {
    // C6 resistance level
    bleResistanceLevelIBD = (*(uint16_t*)(pData+idx));
    idx += 2;
  }
  if (flags & 0x40) {
    // C7 instantaneous power
    bleInstantaneousPowerIBD = (*(uint16_t*)(pData+idx));
    idx += 2;
  }
  if (flags & 0x80) {
    // C8 average power - skip
    idx += 2;
  }
  if (flags & 0x100) {
    // C9 expended energy - skip
    idx += 5;
  }
  if (flags & 0x200) {
    // C10 hear rate - skip
    idx += 1;
  }
  if (flags & 0x400) {
    // C11 metabolic equivalent - skip
    idx += 1;
  }
  if (flags & 0x800) {
    // C12 elapsed time
    bleElapsedTime =  *(uint16_t*)(pData+idx);
  }

  // update the lights
  updateLights();
} // callbackIndoorBikeData


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    BLEUUID uuidSvcFitnessMachine((uint16_t)0x1826);
    uuidSvcFitnessMachine.to128();
    // find any indoor trainer with the fitness machine service advertised
    // because we will be using the indoor bike data characteristic to get power/cadence
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(uuidSvcFitnessMachine)) {
      Serial.println("found indoor trainer");
      BLEDevice::getScan()->stop();
      IndoorTrainerDevice = new BLEAdvertisedDevice (advertisedDevice);
      bleStatus = BLE_SCANNED;
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks

class MyBLEClientCallbacks : public BLEClientCallbacks {
public:
 ~MyBLEClientCallbacks() {};
  void onConnect(BLEClient *pClient){
    Serial.println("BLE client connected!");
  }
  void onDisconnect(BLEClient *pClient) {
    Serial.println("BLE client disconnected!!!");
    // TODO : restore connection!
  }
}; // MyBLEClientCallbacks

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  // setup pixel string
  Serial.println("Init pixels...");

  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  // before connecting BLE set pixels to a light white
  // when BLE is connected the indoor bike data will determine color size
  for (uint8_t i=0;i<NUMPIXELS;i++) {
    pixels.setPixelColor(i,pixels.Color(10,10,10));
  }
  pixels.show();
  

  BLEDevice::init("");
  //02/10/2018
  Serial.println(ESP.getFreeHeap());

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  bleStatus = BLE_SCANNING;
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  // NEW : watoeta?
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->start(30); // same as pBLEScan->start(30, false);
  
} // setup

void activateNotify(BLERemoteCharacteristic *pChar) {

  uint16_t val_2902;
  BLEUUID uuid2902((uint16_t) 0x2902);
  uuid2902.to128();

  BLERemoteDescriptor* pDescriptor2902 = pChar->getDescriptor(uuid2902);
  if (pDescriptor2902 == nullptr) { 
      Serial.println("ERROR met pRemoteDescriptor");
  }
  else {
    uint8_t data[2] = {0x3,0x0};
    pDescriptor2902->writeValue(data, 2, false);
    val_2902 = pDescriptor2902->readUInt16();
    if (val_2902 != 3)
      Serial.printf("writing val=3 to descriptor 2902 failed: %d",val_2902);
  }   
} // activateNotify

int initServicesAndCharacteristics() {
  
  // use BLEUUID instances to find the services / characteristics pointers from the maps
  BLEUUID uuidSvcCyclingSpeedCadence((uint16_t)0x1816);
  BLEUUID uuidSvcCyclingPower((uint16_t)0x1818);
  BLEUUID uuidSvcFitnessMachine((uint16_t)0x1826);
  BLEUUID uuidCharCyclingSpeedCadenceMeasurement((uint16_t)0x2A5B); // UUID 0x2A5B
  BLEUUID uuidCharCyclingPowerMeasurement((uint16_t)0x2A63); // UUID 0x2A63
  BLEUUID uuidCharIndoorBikeData((uint16_t)0x2AD2); // UUID 0x2AD2
  BLEUUID uuidCharTrainingStatus((uint16_t)0x2AD3); // UUID 0x2AD3
  BLEUUID uuidCharFitnessMachineStatus((uint16_t)0x2ADA); // UUID 0x2ADA
  BLEUUID uuidCharFitnessMachineControlPoint((uint16_t)0x2AD9); // UUID 0x2AD9
  uuidSvcCyclingSpeedCadence.to128();
  uuidSvcCyclingPower.to128();
  uuidSvcFitnessMachine.to128();
  uuidCharCyclingSpeedCadenceMeasurement.to128();
  uuidCharCyclingPowerMeasurement.to128();
  uuidCharIndoorBikeData.to128();
  uuidCharTrainingStatus.to128();
  uuidCharFitnessMachineStatus.to128();
  uuidCharFitnessMachineControlPoint.to128();

  bleClient = BLEDevice::createClient();
  if (!bleClient)
    return -1;
  Serial.println(" - Created client");

  // Connect to the remote BLE Server.
  bool retval =  bleClient->connect(IndoorTrainerDevice);
  if (!retval)
  {
    Serial.println("failed to connect to direto");
    return -1;
  }
  Serial.println(" - Connected to server"); // the direto
  // 1. find services we are interested in
  std::map<std::string, BLERemoteService*> *servicesMap;
  BLERemoteService *pSvc;
  servicesMap = bleClient->getServices();

  for (auto &myPair : *servicesMap){
    pSvc = myPair.second;
    if (pSvc->getUUID().equals(uuidSvcCyclingSpeedCadence)) {
      pSvcCyclingSpeedCadence = pSvc;
      Serial.println("found Cycling Speed & Cadence service");
    }
    else if (pSvc->getUUID().equals(uuidSvcCyclingPower)) {
      pSvcCyclingPower = pSvc;
      Serial.println("found Cycling Power service");
    }
    else if (pSvc->getUUID().equals(uuidSvcFitnessMachine)) {
      pSvcFitnessMachine = pSvc;
      Serial.println("found Fitness Machine service");
    }
  }
  
  // 2. find characteristics we are interested in 
  std::map<std::string, BLERemoteCharacteristic *> *characteristicsMap;
  BLERemoteCharacteristic *pChar;

  // Cycling Speed & Cadence
  pSvc = pSvcCyclingSpeedCadence;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharCyclingSpeedCadenceMeasurement)) {
        pCharCyclingSpeedCadenceMeasurement = pChar;
        Serial.println("found Cycling Speed & Cadence Measurement Characteristic");
      }
    }
   }
   
  // Cycling Power
  pSvc = pSvcCyclingPower;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharCyclingPowerMeasurement)) {
        pCharCyclingPowerMeasurement = pChar;
        Serial.println("found Cycling Power Measurement Characteristic");
      }
    }
   }
   
  // Fitness Machine
  pSvc = pSvcFitnessMachine;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharIndoorBikeData)) {
        pCharIndoorBikeData = pChar;
        Serial.println("found Indoor Bike Data Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharTrainingStatus)) {
        pCharTrainingStatus = pChar;
        Serial.println("found Training Status Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharFitnessMachineStatus)) {
        pCharFitnessMachineStatus = pChar;
        Serial.println("found Fitness Machine Status Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharFitnessMachineControlPoint)) {
        pCharFitnessMachineControlPoint = pChar;
        Serial.println("found  Fitness Machine Control Point Characteristic");
      }
    }
   }
   
   // 3. activate notifies
   if (pCharIndoorBikeData) {
     Serial.println("setup ntf for ibd");
     activateNotify(pCharIndoorBikeData);
     pCharIndoorBikeData->registerForNotify(callbackIndoorBikeData);
   }

   Serial.println("initServicesAndCharacteristics completed!");
   return 0;
 
} // initServicesAndCharacteristics

void loop() {
  if (bleStatus == BLE_NOT_CONNECTED) {
    // todo; als we ergens detecteren dat de connectie weg is, gaan we proberen opnieuw te connecteren
  }
  else if (bleStatus == BLE_SCANNING) {
  
  }
  else if (bleStatus == BLE_SCANNED) {
    // let's connect & do initial setup
    Serial.print("Connecting to "); 
    Serial.println(IndoorTrainerDevice->getAddress().toString().c_str());
    delay(2000);
    int retval = initServicesAndCharacteristics();
    if (retval != 0) {
      delay(5000);
      // try again in 5s
    }
    else {
      Serial.println("BLE setup done!");
      bleStatus = BLE_CONNECTED;
    }
    
  }
  else if (bleStatus == BLE_CONNECTED) {
    // any repeated actions here;
  }

} // End of loop
