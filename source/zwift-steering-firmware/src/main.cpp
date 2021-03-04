/*

//Zwift steering demo code
//Takes ADC reading from pin 32 and converts to an angle between -40 and +40 and transmits to Zwift via BLE

  //Based on BLE Arduino for ESP32 examples (Kolban et al.)
  //Keith Wakeham's explanation https://www.youtube.com/watch?v=BPVFjz5zD4g
  //Andy's demo code: https://github.com/fiveohhh/zwift-steerer/

  //Code written in VSCode with PlatformIO for a Lolin32 Lite
  //Should work in Arduino IDE if the #include <Arduino.h> is removed

  //Tested using Zwift on Android (Galaxy A5 2017)

    * Copyright 2020 Peter Everett
    * v1.0 Sep 2020 - Initial version
    * 
    * This work is licensed the GNU General Public License v3 (GPL-3)

  // =========================
  Gert van 't Slot
  v1 - februari 2021
  - Simulate steering
  - Use ADC 0

*/

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define STEERING_DEVICE_UUID     "347b0001-7635-408b-8918-8ff3949ce592"
#define STEERING_ANGLE_CHAR_UUID "347b0030-7635-408b-8918-8ff3949ce592"  //notify
#define STEERING_RX_CHAR_UUID    "347b0031-7635-408b-8918-8ff3949ce592"  //write
#define STEERING_TX_CHAR_UUID    "347b0032-7635-408b-8918-8ff3949ce592"  //indicate
/*
//These charateristics are present on the Sterzo but aren't necessary for communication with Zwift
#define STEERING_POWER_CHAR_UUID "347b0012-7635-408b-8918-8ff3949ce592"     //write
#define STEERING_UNKNOWN2_CHAR_UUID "347b0013-7635-408b-8918-8ff3949ce592"  //value 0xFF, read
#define STEERING_UNKNOWN3_CHAR_UUID "347b0014-7635-408b-8918-8ff3949ce592"  //value 0xFF, notify
#define STEERING_UNKNOWN4_CHAR_UUID "347b0019-7635-408b-8918-8ff3949ce592"  //value x0FF, read
*/

#define HANDSHAKE_DELAY 125
#define DATA_DELAY 100
#define BLE_COOLDOWN_DELAY 50

#define POT A0

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool auth = false;

float angle = 20;
int FF = 0xFF;
uint8_t authChallenge[4] = {0x03, 0x10, 0xff, 0xff};
uint8_t authSuccess[3] = {0x03, 0x11, 0xff};

BLEServer* pServer = NULL;
BLECharacteristic* pAngle = NULL;
/*
//These charateristics are present on the Sterzo but aren't necessary for communication with Zwift
BLECharacteristic* pPwr = NULL;
BLECharacteristic* pU2 = NULL;
BLECharacteristic* pU3 = NULL;
BLECharacteristic* pU4 = NULL;
*/
BLECharacteristic* pRx = NULL;
BLECharacteristic* pTx = NULL;

//BLEAdvertisementData advert;
//BLEAdvertisementData scan_response;
BLEAdvertising* pAdvertising;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

float readAngle() {
    int potVal = analogRead(POT);
    // Serial.println(potVal);
    if (potVal < 1024) {
        return -40;
    } else {
        if (potVal > 3096) {
            return 41;
        } else {
            return (float)((potVal / 25) - 80);
        }
    }
}

void sendAngleToZwift(float angle) {
    //Connected to Zwift so read the potentiometer and start transmitting the angle
    Serial.print("Transmitting angle: ");
    Serial.println(angle);
    pAngle->setValue(angle);
    pAngle->notify();
    delay(DATA_DELAY);
}

void authenticateDevice() {
    //Not connected to Zwift so start the connectin process
    pTx->setValue(FF);
    pTx->indicate();
    //Do the handshaking
    std::string rxValue = pRx->getValue();
    if (rxValue.length() == 0) {
        Serial.println("No data received");
        delay(HANDSHAKE_DELAY);
    } else {
        Serial.print("Handshaking....");
        if (rxValue[0] == 0x03 && rxValue[1] == 0x10) {
            delay(HANDSHAKE_DELAY);
            //send 0x0310FFFF (the last two octets can be anything)
            pTx->setValue(authChallenge, 4);
            pTx->indicate();
            //Zwift will now send 4 bytes as a response, which start with 0x3111
            //We don't really care what it is as long as we get a response
            delay(HANDSHAKE_DELAY);
            rxValue = pRx->getValue();
            if (rxValue.length() == 4) {
                //connected, so send 0x0311ff
                delay(250);
                pTx->setValue(authSuccess, 3);
                pTx->indicate();
                auth = true;
                Serial.println("Success!");
            }
        }
    }
}

void setup() {
    //setup pins for Pot
    pinMode(POT, INPUT);

    Serial.begin(115200);
    //Setup BLE
    Serial.println("Creating BLE server...");
    BLEDevice::init("Gert's Stuur");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    Serial.println("Define service...");
    BLEService* pService = pServer->createService(STEERING_DEVICE_UUID);

    // Create BLE Characteristics
    Serial.println("Define characteristics");
    //The Sterzo includes all of these characteristics, but you only need the Tx and Rx (for the handshaking) and the steerer angle sensor
    /*
  pPwr = pService->createCharacteristic(STEERING_POWER_CHAR_UUID,BLECharacteristic::PROPERTY_WRITE);
  pPwr->addDescriptor(new BLE2902());
  pU2 = pService->createCharacteristic(STEERING_UNKNOWN2_CHAR_UUID,BLECharacteristic::PROPERTY_READ);
  pU2->addDescriptor(new BLE2902());
  pU3 = pService->createCharacteristic(STEERING_UNKNOWN3_CHAR_UUID,BLECharacteristic::PROPERTY_NOTIFY);
  pU3->addDescriptor(new BLE2902());
  pU4 = pService->createCharacteristic(STEERING_UNKNOWN4_CHAR_UUID,BLECharacteristic::PROPERTY_READ);
  pU4->addDescriptor(new BLE2902());*/

    pTx = pService->createCharacteristic(STEERING_TX_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_READ);
    pTx->addDescriptor(new BLE2902());
    pRx = pService->createCharacteristic(STEERING_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRx->addDescriptor(new BLE2902());
    pAngle = pService->createCharacteristic(STEERING_ANGLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pAngle->addDescriptor(new BLE2902());

    // Start the service
    Serial.println("Staring BLE service...");
    pService->start();

    // Start advertising
    // Zwift only shows the steering button when the service is advertised
    Serial.println("Define the advertiser...");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->addServiceUUID(STEERING_DEVICE_UUID);
    pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
    Serial.println("Starting advertiser...");
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

void loop() {
    if (deviceConnected) {
        if (auth) {
            angle = readAngle();
            sendAngleToZwift(angle);
        } else {
            authenticateDevice();
        }
        delay(BLE_COOLDOWN_DELAY);  //small delay so BLE stack doesn't get overloaded
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(HANDSHAKE_DELAY * 2);   // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising();  // restart advertising
        Serial.println("Nothing connected, start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("Connecting...");
    }
    if (!deviceConnected) {
    }
}
