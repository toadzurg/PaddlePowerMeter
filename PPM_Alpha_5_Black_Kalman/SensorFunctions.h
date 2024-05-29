
#ifndef SENSORFUNCTIONS_H
#define SENSORFUNCTIONS_H

#include <Wire.h>
#include "SparkFunLSM6DSO.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

extern LSM6DSO myIMU;
extern NAU7802 myScale;

void setupSensors();
void updateSensors();

#endif
