
#ifndef BLEFUNCTIONS_H
#define BLEFUNCTIONS_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define ppmService_UUID        "1818" //Cycling Power Service
#define ppmMeasurement_UUID    "2A63" //Cycling Power Measurement
#define ppmLocation_UUID       "2A5D" //Cycling Power Sensor Location
#define ppmFeature_UUID        "2A65" //Cycling Power Feature

void setupBLE();
void updateBLE();

#endif
