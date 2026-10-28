// START HERE: Arduino calls setup() once, then calls loop() repeatedly.
// The details live in FlightApp.cpp; this file only starts the application.
#include "FlightApp.h"

// This object owns the receiver, sensors, controllers and motor manager.
itc::FlightApp vehicle;

void setup() {
  vehicle.begin();
}

void loop() {
  vehicle.update();
}
