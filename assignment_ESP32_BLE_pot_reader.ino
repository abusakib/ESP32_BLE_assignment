#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer;
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint32_t ADC_Value = 0; // ADC Reading Voltage value
int ADC_pin = 36;
const float referenceVoltage = 3300.0; 
const int adcResolution = 12; 
unsigned long prvMillis;

#define SERVICE_UUID  "00001523-1212-EFDE-1523-785FEABCD123"
#define CHARACTERISTIC_UUID  "00001524-1212-EFDE-1523-785FEABCD123"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");

    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");

    }
};

void setup() {
  Serial.begin(115200);
  analogReadResolution(adcResolution); 
  BLEDevice::init("POT_Reader");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection...");


}

void loop() {
  if (deviceConnected) {
    uint32_t ADC_Value = read_ADC();
    unsigned long curMillis = millis();
    if ((curMillis - prvMillis) >= 5000) {
      pCharacteristic->setValue((String(ADC_Value)+ " mV").c_str());
      pCharacteristic->notify();
      Serial.println("sent value: " + String(ADC_Value));
      prvMillis = curMillis;
    }
  }
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising(); // Advertising starts when not connected
    Serial.println("Waiting for a client connection...");
    oldDeviceConnected = deviceConnected;
  }
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

uint32_t read_ADC() {
  int rawValue = analogRead(ADC_pin); 
  float voltage = (rawValue / (float)(1 << adcResolution)) * referenceVoltage; 
  return (uint32_t)voltage;
}
