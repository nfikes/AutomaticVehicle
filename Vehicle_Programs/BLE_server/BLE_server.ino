// Last update 11/22/23
//
// Program for the ESP32-WROVER-E BLE compatible device.
// Handles traffic from the BLE delegate as the peripheral to relay data to the NANO for motor control.
// Handles light switching using a level controller and digital switches.
//
// By Nathan Fikes


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Set up these as the Service UUID and the Characteristic UUID. These are needed for the BLE delegatem and to define the server set up by this BLE device.
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331915c"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Dont use pin 17, 16
#define LEFT_PIN 18
#define RIGHT_PIN 5
#define MANUAL_PIN 19
#define AUTO_PIN 4

// Sends some serial to show when connected, and when disconnected start advertising again. This allows us to keep the board on so we don't need to restart it to have it advertise.
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected.");
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected, restarting advertising.");
    BLEDevice::startAdvertising();  // Restart advertising after a client disconnects
  }
};

// When the characteristic is changed we can grab that value and send it to all the serial lines, both the one back to the computer, and the one over to NANO.
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // This will be called when a write operation occurs
    std::string value = pCharacteristic->getValue();
    String incoming_string = value.c_str();
    String incoming_value = incoming_string.substring(1);
    Serial.println(incoming_string);    // Back to computer
    Serial1.print(incoming_string);     // To NANO
    Serial1.print("\n");

    switch (incoming_string[0]) {
      case 'L':
        {
          if(incoming_value.toInt() == 0) {
            digitalWrite(LEFT_PIN, HIGH);
            break;
          } else {
            digitalWrite(LEFT_PIN, LOW);
            break;
          }
        }
      case 'R':
        {
          if(incoming_value.toInt() == 0) {
            digitalWrite(RIGHT_PIN, HIGH);
            break;
          } else {
            digitalWrite(RIGHT_PIN, LOW);
            break;
          }
        }
      case 'M':
        {
          if(incoming_value.toInt() == 0) {
            digitalWrite(MANUAL_PIN, HIGH);
            break;
          } else {
            digitalWrite(MANUAL_PIN, LOW);
            break;
          }
        }
      case 'A':
        {
          if(incoming_value.toInt() == 1) {
            digitalWrite(AUTO_PIN, HIGH);
            break;
          } else {
            digitalWrite(AUTO_PIN, LOW);
            break;
          }
        }
      default:
        { 
        }
    }

  }
};

// Start up the server with a couple things such as the server name, the BAUD, the characteristic properties, callbacks, etc.
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, -1, 23);

  Serial.println("Starting BLE work!");

  BLEDevice::init("AutoVehicleCont自動車制御装置");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->setValue("Hello");
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Setup complete.");

  pinMode(LEFT_PIN, OUTPUT);    // Left Lights [Good]
  pinMode(RIGHT_PIN, OUTPUT);    // Right Lights [Good]
  pinMode(MANUAL_PIN, OUTPUT);     // Manual [Good]
  pinMode(AUTO_PIN, OUTPUT);    // Auto
}

// Repeat on a delay.
void loop() {
  delay(2000);
}