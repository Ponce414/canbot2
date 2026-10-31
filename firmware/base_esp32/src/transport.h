// transport.h — how commands reach the DriveController and measurements leave it.
//
// There are two implementations, and each build includes exactly one of them
// (see build_src_filter in platformio.ini):
//   transport_text.cpp      env:bringup  — human-typeable text over USB, for bench tests
//   transport_microros.cpp  env:microros — ROS 2 topics through the micro-ROS agent on the Pi
//
// Both talk to the controller only through its public methods. So moving from bench
// testing to ROS changes how commands *arrive*, never how the wheels are *controlled*.
#pragma once

class DriveController;

namespace transport {
void begin(DriveController& drive);  // called once from setup(), after the controller starts
void spin();                         // called from loop(), forever
}  // namespace transport
