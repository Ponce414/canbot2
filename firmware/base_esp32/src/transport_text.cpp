// transport_text.cpp — bring-up transport: line-based text commands over native USB.
// Protocol reference: lib/canbot_core/src/text_protocol.h, or send "?" to the board.
#include <Arduino.h>

#include "config.h"
#include "drive_controller.h"
#include "text_protocol.h"
#include "transport.h"

namespace {

DriveController* g_drive = nullptr;
char g_line[canbot::kMaxLineLength];
size_t g_len = 0;
bool g_overflow = false;
uint32_t g_telemetry_period_ms = 0;
uint32_t g_last_telemetry_ms = 0;

// Write a whole line in one call so a reply and a telemetry frame can never interleave
// mid-line. If the host isn't reading (no terminal open), drop the line instead of
// blocking: USB CDC writes can otherwise wait on a full buffer.
void sendLine(const char* text) {
  const size_t n = strlen(text);
  if (Serial.availableForWrite() < static_cast<int>(n + 1)) return;
  Serial.write(reinterpret_cast<const uint8_t*>(text), n);
  Serial.write('\n');
}

void printConfig() {
  char buf[160];
  sendLine("# canbot2 base firmware, bring-up transport");
  snprintf(buf, sizeof(buf), "# counts/rev=%.1f  max=%.1f rad/s  loop=%lu ms  watchdog=%lu ms",
           cfg::kCountsPerWheelRev, cfg::kMaxWheelSpeedRadS, (unsigned long)cfg::kControlPeriodMs,
           (unsigned long)g_drive->watchdogTimeout());
  sendLine(buf);
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    const canbot::PidGains g = g_drive->gains(i);
    snprintf(buf, sizeof(buf), "# %s gains: kp=%.4f ki=%.4f kff=%.4f kstatic=%.4f  invert motor=%d enc=%d",
             cfg::kWheelLabel[i], g.kp, g.ki, g.kff, g.kstatic, cfg::kMotorInvert[i], cfg::kEncoderInvert[i]);
    sendLine(buf);
  }
}

void printHelp() {
  sendLine("# V,fl,fr,rl,rr          wheel speed targets, rad/s");
  sendLine("# D,fl,fr,rl,rr          open-loop duty -1..1");
  sendLine("# S  stop   C  clear faults   Z  zero encoders   ?  help");
  sendLine("# G,w,kp,ki,kff,kstatic  gains (w = 0..3, or -1 for all)");
  sendLine("# T,hz                   telemetry rate 0..100 (0 = off)");
  sendLine("# W,ms                   watchdog 50..5000 ms (commands must repeat faster than this)");
  sendLine("# telemetry: T,ms,cnt x4,vel x4,target x4,duty x4,faults,battery_v,loop_us_max");
  printConfig();
}

void sendTelemetry() {
  DriveState s;
  g_drive->getState(s);
  char buf[320];
  int n = snprintf(buf, sizeof(buf), "T,%lu", (unsigned long)s.stamp_ms);
  for (int i = 0; i < cfg::kNumWheels; ++i) n += snprintf(buf + n, sizeof(buf) - n, ",%lld", (long long)s.counts[i]);
  for (int i = 0; i < cfg::kNumWheels; ++i) n += snprintf(buf + n, sizeof(buf) - n, ",%.3f", s.velocity_rad_s[i]);
  for (int i = 0; i < cfg::kNumWheels; ++i) n += snprintf(buf + n, sizeof(buf) - n, ",%.3f", s.target_rad_s[i]);
  for (int i = 0; i < cfg::kNumWheels; ++i) n += snprintf(buf + n, sizeof(buf) - n, ",%.3f", s.duty[i]);
  snprintf(buf + n, sizeof(buf) - n, ",%u,%.2f,%lu", s.faults, s.battery_v, (unsigned long)s.loop_us_max);
  sendLine(buf);
}

void handleLine(const char* line) {
  canbot::Command cmd;
  const canbot::ParseError err = canbot::parseCommand(line, cmd);
  if (err == canbot::ParseError::Empty) return;
  if (err != canbot::ParseError::Ok) {
    char buf[48];
    snprintf(buf, sizeof(buf), "ERR,%s", canbot::parseErrorName(err));
    sendLine(buf);
    return;
  }

  switch (cmd.type) {
    case canbot::CmdType::Velocity: g_drive->commandVelocity(cmd.values); break;
    case canbot::CmdType::Duty: g_drive->commandDuty(cmd.values); break;
    case canbot::CmdType::Stop: g_drive->stop(); break;
    case canbot::CmdType::ClearFaults: g_drive->clearFaults(); break;
    case canbot::CmdType::ZeroEncoders: g_drive->zeroEncoders(); break;
    case canbot::CmdType::Gains: {
      const canbot::PidGains g{cmd.values[0], cmd.values[1], cmd.values[2], cmd.values[3]};
      g_drive->setGains(cmd.wheel, g);
      break;
    }
    case canbot::CmdType::Telemetry:
      g_telemetry_period_ms = cmd.integer == 0 ? 0 : 1000u / cmd.integer;
      break;
    case canbot::CmdType::Watchdog: g_drive->setWatchdogTimeout(cmd.integer); break;
    case canbot::CmdType::Help: printHelp(); return;  // help is its own reply
  }
  sendLine("OK");
}

}  // namespace

namespace transport {

void begin(DriveController& drive) {
  g_drive = &drive;
  // A telemetry line is ~170 bytes; the default 256-byte USB transmit buffer would fit only
  // one, so a reply sent right after it could be dropped by sendLine(). 1 KB holds several.
  // Must be set before begin().
  Serial.setTxBufferSize(1024);
  // On the S3's native USB port the baud rate is ignored (USB has its own speed), but
  // Serial.begin() still sets up the buffers.
  Serial.begin(115200);
  delay(200);
  printHelp();
}

void spin() {
  // Non-blocking line assembly: take whatever bytes have arrived, act on complete lines.
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_overflow) {
        sendLine("ERR,too_long");
      } else if (g_len > 0) {
        g_line[g_len] = '\0';
        handleLine(g_line);
      }
      g_len = 0;
      g_overflow = false;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    } else {
      g_overflow = true;  // keep discarding until the end of this line
    }
  }

  const uint32_t now = millis();
  if (g_telemetry_period_ms > 0 && now - g_last_telemetry_ms >= g_telemetry_period_ms) {
    g_last_telemetry_ms = now;
    sendTelemetry();
  }
  delay(1);  // yield; nothing here is time-critical
}

}  // namespace transport
