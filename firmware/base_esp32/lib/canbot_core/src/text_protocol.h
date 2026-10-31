// text_protocol.h — parser for the human-typeable bring-up protocol.
//
// WHY A TEXT PROTOCOL AT ALL, WHEN THE ROBOT WILL USE micro-ROS
//   Bring-up is about isolating one unknown at a time. With plain text over USB you can
//   drive one wheel from a serial monitor with nothing else running: no ROS, no agent, no
//   Pi. Once a wheel tracks its target here, micro-ROS replaces only the transport; the
//   control code underneath is byte-for-byte the same.
//
// FORMAT: one command per line, comma-separated, newline-terminated. Wheel order is
//   always FL, FR, RL, RR. Replies are "OK" or "ERR,<reason>".
//
//   V,fl,fr,rl,rr          closed-loop wheel speed targets, rad/s
//   D,fl,fr,rl,rr          open-loop duty, -1.0..1.0 (bench tests; no PID)
//   S                      stop: all wheels off, integrals reset
//   C                      clear latched faults (stall)
//   Z                      zero the encoder counts / wheel positions
//   G,w,kp,ki,kff,kstatic  set gains for wheel w (0..3), or all wheels if w = -1
//   T,hz                   telemetry rate, 0..100 Hz (0 = off)
//   W,ms                   command watchdog timeout, 50..5000 ms
//   ?                      print help, gains and configuration
//
// The parser only checks syntax and ranges. Deciding what a command *does* is the
// transport's job; that split keeps this file testable without any hardware.
#pragma once

#include <cstddef>
#include <cstdint>

namespace canbot {

enum class CmdType : uint8_t {
  Velocity,
  Duty,
  Stop,
  ClearFaults,
  ZeroEncoders,
  Gains,
  Telemetry,
  Watchdog,
  Help,
};

enum class ParseError : uint8_t {
  Ok,
  Empty,
  TooLong,
  UnknownCommand,
  WrongArgCount,
  BadNumber,
  OutOfRange,
};

struct Command {
  CmdType type = CmdType::Stop;
  float values[4] = {0, 0, 0, 0};  // V/D: the four wheels.  G: kp, ki, kff, kstatic
  int wheel = -1;                  // G only
  uint32_t integer = 0;            // T: Hz.  W: ms
};

constexpr size_t kMaxLineLength = 96;
constexpr float kMaxVelocityCommand = 100.0f;  // rad/s sanity bound; the controller clamps lower

// Parses one line (without its trailing newline). Leading/trailing spaces are ignored and
// the command letter is case-insensitive. Returns ParseError::Ok and fills `out` on success.
ParseError parseCommand(const char* line, Command& out);

const char* parseErrorName(ParseError e);

}  // namespace canbot
