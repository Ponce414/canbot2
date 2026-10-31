// main.cpp — canbot2 drive base firmware (ESP32-S3).
//
// Deliberately tiny: start the 100 Hz control task, then hand loop() to the transport.
// Everything time-critical lives in DriveController's own FreeRTOS task, so nothing the
// transport does (waiting on USB, reconnecting to the micro-ROS agent) can delay a
// control step.
#include <Arduino.h>

#include "drive_controller.h"
#include "transport.h"

static DriveController drive;

void setup() {
  // Controller first: it drives every motor output to 0% before anything else runs.
  drive.begin();
  transport::begin(drive);
}

void loop() { transport::spin(); }
