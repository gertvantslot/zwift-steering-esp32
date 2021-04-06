/*

// Zwift steering code
// * PlatformIO
// * ESP32 - MH-ET Life: 
// - https://docs.platformio.org/en/latest/boards/espressif32/mhetesp32devkit.html 
// - https://forum.mhetlive.com/topic/2/mh-et-live-esp32-devkit


// Takes ADC reading from pin 36 and converts to an angle between -40 and +40 and transmits to Zwift via BLE

  //Based on BLE Arduino for ESP32 examples (Kolban et al.)
  //Keith Wakeham's explanation https://www.youtube.com/watch?v=BPVFjz5zD4g
  //Andy's demo code: https://github.com/fiveohhh/zwift-steerer/

  //Tested using Zwift on Android (Galaxy A5 2017)

    * Copyright 2020 Peter Everett
    * v1.0 Sep 2020 - Initial version
    * 
    * This work is licensed the GNU General Public License v3 (GPL-3)

  // =========================
  Gert van 't Slot
  v1 - februari 2021
  - Calibration mode
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

#define DELAY_HANDSHAKE 125
#define DELAY_HANDSHAKE2 250
#define DELAY_BLE_COOLDOWN 100
#define DELAY_BLINK 250

#define POT A0

#define CH_INDICATOR 15

// Timers
unsigned long timer_data = 0;
unsigned long timer_ble_cooldown = 0;
unsigned long timer_blink = 0;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool auth = false;

boolean error_angle = true;
float angle = 0;
const float angle_average = 250; // Number of measurements to average

// Define ANGLE_CALIBRATE to print the measured angle
// to Serial
#define xANGLE_CALIBRATE

// Next 2 values are used for calibration 
// Set to the raw value measured when steering to max left
const uint16_t angle_min = 1024; // Value of potmeter, when minimum angle = -40°

// Set to the raw value measured when steering to max right
const uint16_t angle_max = 3096; // Value of potmeter, when maximum angle =  40°

// Set to the value you want sent to swift, when steering to max right. 
const float zwift_angle_sensitivity = 40.0; // Default = 40.0°;

const uint16_t angle_range = angle_max - angle_min;

#ifdef ANGLE_CALIBRATE
    // Minimum and maximum values, make sure they are overwritten on first pass
    uint16_t pot_measured_min = UINT16_MAX;
    uint16_t pot_measured_max = 0;
    float angle_measured_min = 180.0f;
    float angle_measured_max = -180;
#endif

// define the return values to Zwift
const float zwift_angle_min = -zwift_angle_sensitivity; 
const float zwift_angle_max =  zwift_angle_sensitivity; 
const float zwift_angle_range = zwift_angle_max - zwift_angle_min; // 40 - (-40) = 80
const float zwift_angle_factor = zwift_angle_range / (float)angle_range;

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
    float retVal = 0.0;

    if (error_angle && potVal > angle_min) {
        // We have input
        error_angle = false;
    }

    if (potVal < angle_min) {
        retVal = zwift_angle_min;
    } else {
        if (potVal > angle_max) {
            retVal = zwift_angle_max;
        } else {
            retVal = zwift_angle_min + (potVal - angle_min) * zwift_angle_factor;
        }
    }

    #ifdef ANGLE_CALIBRATE

    if (potVal < pot_measured_min) pot_measured_min = potVal;
    if (potVal > pot_measured_max) pot_measured_max = potVal;
    if (retVal < angle_measured_min) angle_measured_min = retVal;
    if (retVal > angle_measured_max) angle_measured_max = retVal;


    Serial.print("** ");
    Serial.print(potVal);
    Serial.print(" = ");
    Serial.print(retVal);
    // Serial.println();

    Serial.print("  - Minimum values: ");
    Serial.print(pot_measured_min);
    Serial.print(" - ");
    Serial.print(angle_measured_min);
    // Serial.println();

    Serial.print("  - Maximum values: ");
    Serial.print(pot_measured_max);
    Serial.print(" - ");
    Serial.print(angle_measured_max);
    Serial.println();

    #endif

    return retVal;
}

void handleIndicatorLight() {
    if (millis() - timer_blink < DELAY_BLINK) {
        return;
    }
    timer_blink = millis();

    if (deviceConnected && auth) {
        if (error_angle) {
            // Do some blinking
            ledcWrite(CH_INDICATOR, 255 - ((millis() / DELAY_BLINK) % 4) * 24);
        } else {
            ledcWrite(CH_INDICATOR, map((int)angle, -41, 40, 0, 255));
        }
    } else {
        ledcWrite(CH_INDICATOR, 255 - ((millis() / DELAY_BLINK) % 2) * 64);
    }
}

void sendAngleToZwift(float angle) {
    //Connected to Zwift so read the potentiometer and start transmitting the angle
    pAngle->setValue(angle);
    pAngle->notify();
}

void authenticateDevice() {
    //Not connected to Zwift so start the connectin process
    pTx->setValue(FF);
    pTx->indicate();
    //Do the handshaking
    std::string rxValue = pRx->getValue();
    if (rxValue.length() == 0) {
        Serial.println("No data received");
        delay(DELAY_HANDSHAKE);
    } else {
        Serial.print("Handshaking....");
        if (rxValue[0] == 0x03 && rxValue[1] == 0x10) {
            delay(DELAY_HANDSHAKE);
            //send 0x0310FFFF (the last two octets can be anything)
            pTx->setValue(authChallenge, 4);
            pTx->indicate();
            //Zwift will now send 4 bytes as a response, which start with 0x3111
            //We don't really care what it is as long as we get a response
            delay(DELAY_HANDSHAKE);
            rxValue = pRx->getValue();
            if (rxValue.length() == 4) {
                //connected, so send 0x0311ff
                delay(DELAY_HANDSHAKE2);
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
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);

    //Setup BLE
    Serial.println("Creating BLE server...");
    BLEDevice::init("STUUR-GERT");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    Serial.println("Define service...");
    BLEService* pService = pServer->createService(STEERING_DEVICE_UUID);

    // Create BLE Characteristics
    Serial.println("Define characteristics");

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

    // Use LED_BUILTIN to give feedback
    ledcSetup(CH_INDICATOR, 5000, 8);
    ledcAttachPin(LED_BUILTIN, CH_INDICATOR);
}

void loop() {

    // Read angle, reduce noise
    angle += (readAngle() - angle) / angle_average;

    handleIndicatorLight();

    // small interval so BLE stack doesn't get overloaded
    if (millis() - timer_ble_cooldown > DELAY_BLE_COOLDOWN) {
        timer_ble_cooldown = millis();
        
        if (deviceConnected) {
            if (auth) {
                sendAngleToZwift(angle);
            } else {
                authenticateDevice();
            }
        }
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(DELAY_HANDSHAKE2);   // give the bluetooth stack the chance to get things ready
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
