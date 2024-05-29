
#include "MainFunctions.h"
#include "BLEFunctions.h"
#include "SensorFunctions.h"
#include "PreferencesFunctions.h"

void setupMain() {
    setupBLE();
    setupSensors();
    setupPreferences();
}

void loopMain() {
    updateBLE();
    updateSensors();
}
