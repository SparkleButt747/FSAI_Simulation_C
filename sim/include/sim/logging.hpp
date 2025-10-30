#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace fsai::sim::log {

enum class Level {
  kInfo,
  kWarning,
  kError,
};

struct Entry {
  Level level;
  std::string formatted;
};

// Append a message to the in-memory log buffer. Optionally mirror to stdout.
void Log(Level level, std::string_view message, bool also_stdout = false);

// Convenience helpers for common log levels.
void LogInfo(std::string_view message);
void LogWarning(std::string_view message);
void LogError(std::string_view message);

// Variants that also print to stdout immediately.
void LogInfoToStdout(std::string_view message);

// printf-style helpers for formatting messages.
void Logf(Level level, const char* fmt, ...);
void LogfToStdout(Level level, const char* fmt, ...);

// Retrieve a snapshot of the buffered log entries.
std::vector<Entry> Snapshot();

// Clear all buffered log entries.
void Clear();

// Returns true if the UI should scroll to the newest entry.
bool ConsumeScrollRequest();

}  // namespace fsai::sim::log
