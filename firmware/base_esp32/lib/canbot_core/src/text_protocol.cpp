#include "text_protocol.h"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace canbot {

namespace {

constexpr int kMaxFields = 8;

// Split `buf` in place on commas. Unlike strtok, this keeps empty fields ("V,1,,2,3"
// yields an empty field, which then fails number parsing instead of silently shifting
// the remaining values into the wrong wheels) and holds no hidden static state.
int splitFields(char* buf, char* fields[], int max_fields) {
  int n = 0;
  char* start = buf;
  for (char* p = buf;; ++p) {
    if (*p == ',' || *p == '\0') {
      const bool end = (*p == '\0');
      *p = '\0';
      if (n == max_fields) return max_fields + 1;  // too many fields
      fields[n++] = start;
      if (end) break;
      start = p + 1;
    }
  }
  return n;
}

// Trim spaces in place; returns the new start pointer.
char* trim(char* s) {
  while (*s && std::isspace(static_cast<unsigned char>(*s))) ++s;
  char* end = s + std::strlen(s);
  while (end > s && std::isspace(static_cast<unsigned char>(end[-1]))) --end;
  *end = '\0';
  return s;
}

// strtof alone accepts "1.5abc" as 1.5. Checking that endptr reached the end of the field
// rejects typos instead of quietly running a wheel at the wrong speed.
bool parseFloat(char* field, float& out) {
  char* s = trim(field);
  if (*s == '\0') return false;
  char* end = nullptr;
  errno = 0;
  const float v = std::strtof(s, &end);
  if (errno != 0 || end == s || *end != '\0' || !std::isfinite(v)) return false;
  out = v;
  return true;
}

bool parseInt(char* field, long& out) {
  char* s = trim(field);
  if (*s == '\0') return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') return false;
  out = v;
  return true;
}

}  // namespace

ParseError parseCommand(const char* line, Command& out) {
  if (line == nullptr) return ParseError::Empty;
  const size_t len = std::strlen(line);
  if (len >= kMaxLineLength) return ParseError::TooLong;

  char buf[kMaxLineLength];
  std::memcpy(buf, line, len + 1);
  char* text = trim(buf);
  if (*text == '\0') return ParseError::Empty;

  char* fields[kMaxFields];
  const int n = splitFields(text, fields, kMaxFields);
  if (n > kMaxFields) return ParseError::WrongArgCount;

  char* head = trim(fields[0]);
  if (std::strlen(head) != 1) return ParseError::UnknownCommand;
  const char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(head[0])));
  const int args = n - 1;

  switch (letter) {
    case 'V':
    case 'D': {
      if (args != 4) return ParseError::WrongArgCount;
      const float bound = (letter == 'V') ? kMaxVelocityCommand : 1.0f;
      for (int i = 0; i < 4; ++i) {
        if (!parseFloat(fields[i + 1], out.values[i])) return ParseError::BadNumber;
        if (std::fabs(out.values[i]) > bound) return ParseError::OutOfRange;
      }
      out.type = (letter == 'V') ? CmdType::Velocity : CmdType::Duty;
      return ParseError::Ok;
    }
    case 'S':
    case 'C':
    case 'Z':
    case '?': {
      if (args != 0) return ParseError::WrongArgCount;
      out.type = letter == 'S'   ? CmdType::Stop
                 : letter == 'C' ? CmdType::ClearFaults
                 : letter == 'Z' ? CmdType::ZeroEncoders
                                 : CmdType::Help;
      return ParseError::Ok;
    }
    case 'G': {
      if (args != 5) return ParseError::WrongArgCount;
      long w = 0;
      if (!parseInt(fields[1], w)) return ParseError::BadNumber;
      if (w < -1 || w > 3) return ParseError::OutOfRange;
      for (int i = 0; i < 4; ++i) {
        if (!parseFloat(fields[i + 2], out.values[i])) return ParseError::BadNumber;
        if (out.values[i] < 0.0f || out.values[i] > 10.0f) return ParseError::OutOfRange;
      }
      out.wheel = static_cast<int>(w);
      out.type = CmdType::Gains;
      return ParseError::Ok;
    }
    case 'T':
    case 'W': {
      if (args != 1) return ParseError::WrongArgCount;
      long v = 0;
      if (!parseInt(fields[1], v)) return ParseError::BadNumber;
      // The watchdog has a ceiling on purpose: long enough to type the next command by
      // hand, never long enough to forget a spinning wheel.
      const long lo = (letter == 'T') ? 0 : 50;
      const long hi = (letter == 'T') ? 100 : 5000;
      if (v < lo || v > hi) return ParseError::OutOfRange;
      out.integer = static_cast<uint32_t>(v);
      out.type = (letter == 'T') ? CmdType::Telemetry : CmdType::Watchdog;
      return ParseError::Ok;
    }
    default:
      return ParseError::UnknownCommand;
  }
}

const char* parseErrorName(ParseError e) {
  switch (e) {
    case ParseError::Ok: return "ok";
    case ParseError::Empty: return "empty";
    case ParseError::TooLong: return "too_long";
    case ParseError::UnknownCommand: return "unknown_command";
    case ParseError::WrongArgCount: return "wrong_arg_count";
    case ParseError::BadNumber: return "bad_number";
    case ParseError::OutOfRange: return "out_of_range";
  }
  return "unknown";
}

}  // namespace canbot
