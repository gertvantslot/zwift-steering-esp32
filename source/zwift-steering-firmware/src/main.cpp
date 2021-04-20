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
#include <math.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "settings.h"
#include "calibrate.h"

#define STEERING_DEVICE_UUID     "347b0001-7635-408b-8918-8ff3949ce592"
#define STEERING_ANGLE_CHAR_UUID "347b0030-7635-408b-8918-8ff3949ce592"  //notify
#define STEERING_RX_CHAR_UUID    "347b0031-7635-408b-8918-8ff3949ce592"  //write
#define STEERING_TX_CHAR_UUID    "347b0032-7635-408b-8918-8ff3949ce592"  //indicate

#define DELAY_HANDSHAKE 125
#define DELAY_HANDSHAKE2 250
#define DELAY_BLE_COOLDOWN 100
#define DELAY_BLINK 250

// Channel to use for PWM 
//  indicator on LED_BUILT_IN
#define CH_INDICATOR 15

// Timers
unsigned long timer_data = 0;
unsigned long timer_ble_cooldown = 0;
unsigned long timer_blink = 0;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool authenticated = false;

boolean measurementsReceived = false;
float angle = 0;
const float angle_average = 250; // Number of measurements to average
                                 // Higher = reduce noise, but slower steering

uint16_t potValue_range = potValue_max - potValue_min;

#ifdef DEBUG_TO_SERIAL
    #define pln(X) Serial.println(X)
    #define p(X) Serial.print(X)
#else
    #define pln(X)
    #define p(X)
#endif

#ifdef ANGLE_CALIBRATE
    // Minimum and maximum values, make sure they are overwritten on first pass
    uint16_t pot_measured_min = UINT16_MAX;
    uint16_t pot_measured_max = 0;
    float angle_measured_min = 180.0f;
    float angle_measured_max = -180;

    unsigned long timer_calibrate = 0;
    #define DELAY_CALIBRATE 5000
#endif

#ifdef AUTO_CENTER

    #define AUTOCENTER_SIZE 1024
    #define AUTOCENTER_RESOLUTION 4
    #define AUTOCENTER_EDGE 0x10

    #ifdef AUTO_CENTER_DEBUG
        #define AUTOCENTER_INTERVAL 30000
        #define AUTOCENTER_MAX_ADJUST 0x40
    #else
        #define AUTOCENTER_INTERVAL 60000
        #define AUTOCENTER_MAX_ADJUST 0x20
    #endif

    uint64_t autoCenterMeasurements[AUTOCENTER_SIZE];
    int autoCenterCenter = (potValue_min + potValue_max) / 2; // Initialize to center
    unsigned long autoCenterLastTime = 0;

    void autoCenterStore(uint16_t potVal) {
        // Only when active in the game
        if (!measurementsReceived) return;
        #ifndef AUTO_CENTER_DEBUG
            if (!authenticated) return;
        #endif

        int index = potVal / AUTOCENTER_RESOLUTION;
        autoCenterMeasurements[index]++;
    }

    void autoCenterClear() {
        for(int i = 0; i < AUTOCENTER_SIZE; i++) {
            autoCenterMeasurements[i] = 0;
        }
    }

    // https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter
    // https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter#/media/File:Lorentzian_and_derivative.gif
    // http://www.statistics4u.info/fundstat_eng/cc_savgol_coeff.html
    #define SG_1ST_SIZE 25
    #define SG_1ST_NORM 1300.0F

    #define SG_1ST_SIZE_RNG ((SG_1ST_SIZE - 1) / 2)
    // Coefficient, for 1st derivative : a(i) = i  
    float autoCenterSmoothedFirstDerivative(uint16_t index) {
        float sum = 0;
        for(int i = -SG_1ST_SIZE_RNG; i <= SG_1ST_SIZE_RNG; i++) {
            float sg_coeff = i; // See http://www.statistics4u.info/fundstat_eng/cc_savgol_coeff.html 
            sum += (float)(autoCenterMeasurements[index + i]) * sg_coeff;
        }
        return sum / SG_1ST_NORM;
    }

    uint16_t autoCenterMidPoint() {
        // Get the index of the value that is used the most
        // this will be the direction the user has steered the most of the time
        // and we hope, this will be straight
        // 
        // Method: 
        // For every point, get the (smoothed) 1st derivative
        // Get the index of the highest and lowest value
        // The midpoint will be center between these two
        int index_max = 0;
        int index_min = 0;
        float max = std::numeric_limits<float>::min();
        float min = std::numeric_limits<float>::max();

        bool containsData = false;

        const int edgeMin = (potValue_min / AUTOCENTER_RESOLUTION) + AUTOCENTER_EDGE;
        const int edgeMax = (potValue_max / AUTOCENTER_RESOLUTION) - AUTOCENTER_EDGE;

        p("AutoCenter: Scanning for midpoint [");
        p(edgeMin * AUTOCENTER_RESOLUTION);
        p(" , ");
        p(edgeMax * AUTOCENTER_RESOLUTION);
        pln("]");

        for(int i = edgeMin; i < edgeMax; i++) {
            if (autoCenterMeasurements[i] > 100) {
                containsData = true;
            }
            float smoothed = autoCenterSmoothedFirstDerivative(i);

            if (smoothed > max) {
                max = smoothed;
                index_max = i;
            }            
            if (smoothed < min) {
                min = smoothed;
                index_min = i;
            }
        }

        if (!containsData) {
            // Nothing measured, or steer is on maximum range
            pln("AutoCenter: No data: Midpoint not recalculated");
            return autoCenterCenter; // Current center
        }

        int midPoint = (index_min + index_max) / 2;
        
        p("AutoCenter: Midpoint recalculated: ");
        p(autoCenterCenter);
        p(" => ");
        pln(midPoint * AUTOCENTER_RESOLUTION);

        return (uint16_t)(midPoint * AUTOCENTER_RESOLUTION);
    }

    #ifdef AUTO_CENTER_DEBUG
    void autoCenterPrintGraph() {
        // Graph
        const uint32_t lines = 32;
        const uint32_t values_per_line = AUTOCENTER_SIZE / 32;
        uint64_t values[lines];

        uint64_t max = 0;

        // Dermine scale
        for(int i = 0; i < lines; i++) {
            uint64_t sum = 0;
            for(int j = 0; j < values_per_line; j++) {
                sum += autoCenterMeasurements[(i * values_per_line) + j];
            }
            values[i] = sum;
            if (sum > max) {
                max = sum;
            }
        }

        // float scale = 60.0 / max;

        for(int i = 0; i < lines; i++) {
            p(i * values_per_line * AUTOCENTER_RESOLUTION);
            p(" = ");
            pln(values[i]);
        }
    }
    #endif

    // Calibrate - Determine new midpoint, only once overy X seconds
    void autoCenterCalibrate() {
        if (millis() - autoCenterLastTime < AUTOCENTER_INTERVAL) return;
        autoCenterLastTime = millis();

        pln("AutoCenter: Calibration started");

        #ifdef AUTO_CENTER_DEBUG
        autoCenterPrintGraph();
        #endif

        int newCenter = autoCenterMidPoint();
        if (abs(newCenter - autoCenterCenter) < AUTOCENTER_MAX_ADJUST) {
            autoCenterCenter = newCenter;
        } else {
            if (newCenter < autoCenterCenter) {
                autoCenterCenter -= AUTOCENTER_MAX_ADJUST;
            } else {
                autoCenterCenter += AUTOCENTER_MAX_ADJUST;
            }
        }

        p("Active midpoint: ");
        pln(autoCenterCenter);

        autoCenterClear();
    }

    float autoCenterCorrectedAngle(uint16_t potVal) {
        if (potVal == autoCenterCenter) return 0.0F;
        if (potVal < autoCenterCenter) {
            // left of center = positive value
            // scale from potValue_min - autoCenterCenter === -zwift_angle_min° - 0°

            float range = autoCenterCenter - potValue_min;
            float retVal = ((float)autoCenterCenter - (float)potVal) / range * zwift_angle_sensitivity;
            #ifdef xAUTO_CENTER_DEBUG
            p("AutoCenter: Corrected value: L: ");
            p(potVal);
            p(" => ");
            p(retVal);
            pln();
            #endif
            return retVal;
        } else {
            // right of center = negative value
            float range = potValue_max - autoCenterCenter;
            float retVal = ((float)autoCenterCenter - (float)potVal) / range * zwift_angle_sensitivity;
            #ifdef xAUTO_CENTER_DEBUG
            p("AutoCenter: Corrected value: R: ");
            p(potVal);
            p(" => ");
            p(retVal);
            pln();
            #endif
            return retVal;
        }
    }

#endif

// define the return values to Zwift
const float zwift_angle_min = -zwift_angle_sensitivity; 
const float zwift_angle_max =  zwift_angle_sensitivity; 
const float zwift_angle_range = zwift_angle_max - zwift_angle_min; // 40 - (-40) = 80
float zwift_angle_factor = zwift_angle_range / (float)potValue_range;

int FF = 0xFF;
uint8_t authChallenge[4] = {0x03, 0x10, 0xff, 0xff};
uint8_t authSuccess[3] = {0x03, 0x11, 0xff};

BLEServer* pServer = NULL;
BLECharacteristic* pAngle = NULL;
BLECharacteristic* pRx = NULL;
BLECharacteristic* pTx = NULL;

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
    if (potVal < potValue_error) {
        // Too low for a correct measurement
        return 0.0;
    }
    
    #ifdef xAUTO_CENTER_DEBUG
        potVal = (4096 / 4) + sin(millis() / 60000.0) * 300;
    #endif
    float retVal = 0.0;

    if (!measurementsReceived && potVal > potValue_error) {
        // We have input
        pln("Looks like the system is connected");
        measurementsReceived = true;
    }

    #ifdef AUTO_CALIBRATE

    if (measurementsReceived) {
        if (potVal < potValue_min) {
            potValue_min -= 2; // Decrease, but not too much (error measurements)
            #ifdef AUTO_CENTER
            autoCenterCenter--;
            #endif
            potValue_range = potValue_max - potValue_min;
            zwift_angle_factor = zwift_angle_range / (float)potValue_range;
            pln("AutoCalibrate: <<< Angle Min corrected");
        }
        if (potVal > potValue_max) {
            potValue_max += 2;
            #ifdef AUTO_CENTER
              autoCenterCenter++;
            #endif
            potValue_range = potValue_max - potValue_min;
            zwift_angle_factor = zwift_angle_range / (float)potValue_range;
            pln("AutoCalibrate: >>> Angle Max corrected");
        }
    }

    #endif

    #ifdef AUTO_CENTER 
        autoCenterStore(potVal);
    #endif

    if (potVal < potValue_min) {
        retVal = zwift_angle_min;
    } else {
        if (potVal > potValue_max) {
            retVal = zwift_angle_max;
        } else {
            #ifdef AUTO_CENTER 
            retVal = autoCenterCorrectedAngle(potVal);
            #else
            retVal = zwift_angle_min + (potVal - potValue_min) * zwift_angle_factor;
            #endif
        }
    }

    // Correct for direction
    retVal *= zwift_angle_direction;

    #ifdef ANGLE_CALIBRATE

    if (millis() - timer_calibrate > DELAY_CALIBRATE) {
        timer_calibrate = millis();
        if (potVal < pot_measured_min) pot_measured_min = potVal;
        if (potVal > pot_measured_max) pot_measured_max = potVal;
        if (retVal < angle_measured_min) angle_measured_min = retVal;
        if (retVal > angle_measured_max) angle_measured_max = retVal;

        p("** ");
        p(potVal);
        p(" = ");
        p(retVal);

        p("  - Minimum values: ");
        p(pot_measured_min);
        p(" - ");
        p(angle_measured_min);

        p("  - Maximum values: ");
        p(pot_measured_max);
        p(" - ");
        p(angle_measured_max);
        pln();
    }
    #endif

    return retVal;
}

void handleIndicatorLight() {
    if (millis() - timer_blink < DELAY_BLINK) {
        return;
    }
    timer_blink = millis();

    if (deviceConnected && authenticated) {
        if (!measurementsReceived) {
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
        pln("No data received");
        delay(DELAY_HANDSHAKE);
    } else {
        pln("Handshaking....");
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
                authenticated = true;
                pln("Success!");
            }
        }
    }
}

void setup() {
    //setup pins for Pot
    pinMode(POT, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    #ifdef DEBUG_TO_SERIAL
        Serial.begin(115200);
    #endif

    //Setup BLE
    pln("Creating BLE server...");
    BLEDevice::init(NAME);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    pln("Define service...");
    BLEService* pService = pServer->createService(STEERING_DEVICE_UUID);

    // Create BLE Characteristics
    pln("Define characteristics");

    pTx = pService->createCharacteristic(STEERING_TX_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_READ);
    pTx->addDescriptor(new BLE2902());
    pRx = pService->createCharacteristic(STEERING_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRx->addDescriptor(new BLE2902());
    pAngle = pService->createCharacteristic(STEERING_ANGLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pAngle->addDescriptor(new BLE2902());

    // Start the service
    pln("Staring BLE service...");
    pService->start();

    // Start advertising
    // Zwift only shows the steering button when the service is advertised
    pln("Define the advertiser...");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->addServiceUUID(STEERING_DEVICE_UUID);
    pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
    pln("Starting advertiser...");
    BLEDevice::startAdvertising();
    pln("Waiting a client connection to notify...");

    // Use LED_BUILTIN to give feedback
    ledcSetup(CH_INDICATOR, 5000, 8);
    ledcAttachPin(LED_BUILTIN, CH_INDICATOR);
}

void loop() {
    // Read angle, reduce noise
    angle += (readAngle() - angle) / angle_average;

    #ifdef AUTO_CENTER
        autoCenterCalibrate();
    #endif

    handleIndicatorLight();

    // small interval so BLE stack doesn't get overloaded
    if (millis() - timer_ble_cooldown > DELAY_BLE_COOLDOWN) {
        timer_ble_cooldown = millis();
        
        if (deviceConnected) {
            if (authenticated) {
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
        pln("Nothing connected, start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        pln("Connecting...");
    }
}
