#include <sim/logging.hpp>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <ctime>
#include <mutex>
#include <sstream>
#include <string>

namespace fsai::sim::log {
namespace {
constexpr size_t kMaxEntries = 512;

const char* LevelTag(Level level) {
  switch (level) {
    case Level::kInfo:
      return "INFO";
    case Level::kWarning:
      return "WARN";
    case Level::kError:
      return "ERROR";
  }
  return "INFO";
}

struct LogState {
  std::deque<Entry> entries;
  bool scroll_to_bottom{false};
  std::mutex mutex;
};

LogState& GetState() {
  static LogState state;
  return state;
}

std::string FormatTimestamp(const std::chrono::system_clock::time_point& tp) {
  const auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      tp.time_since_epoch()) % 1000;
  std::ostringstream oss;
  oss << std::put_time(&tm, "%H:%M:%S") << '.' << std::setw(3) << std::setfill('0')
      << ms.count();
  return oss.str();
}

std::string SanitizeMessage(std::string_view message) {
  std::string sanitized(message);
  while (!sanitized.empty() &&
         (sanitized.back() == '\n' || sanitized.back() == '\r')) {
    sanitized.pop_back();
  }
  return sanitized;
}

std::string FormatString(const char* fmt, va_list args) {
  va_list copy;
  va_copy(copy, args);
  const int len = std::vsnprintf(nullptr, 0, fmt, copy);
  va_end(copy);
  if (len <= 0) {
    return {};
  }
  std::string buffer(static_cast<size_t>(len) + 1, '\0');
  std::vsnprintf(buffer.data(), buffer.size(), fmt, args);
  buffer.resize(static_cast<size_t>(len));
  return buffer;
}

void AppendEntry(Level level, std::string_view message, bool also_stdout) {
  std::string sanitized = SanitizeMessage(message);
  const auto now = std::chrono::system_clock::now();
  const std::string timestamp = FormatTimestamp(now);
  std::string formatted = '[' + timestamp + std::string("] [") + LevelTag(level) + "] " +
                          sanitized;

  auto& state = GetState();
  {
    std::scoped_lock lock(state.mutex);
    state.entries.push_back(Entry{level, formatted});
    if (state.entries.size() > kMaxEntries) {
      state.entries.pop_front();
    }
    state.scroll_to_bottom = true;
  }

  if (also_stdout) {
    std::fprintf(stdout, "%s\n", formatted.c_str());
  }
}

}  // namespace

void Log(Level level, std::string_view message, bool also_stdout) {
  AppendEntry(level, message, also_stdout);
}

void LogInfo(std::string_view message) { Log(Level::kInfo, message); }

void LogWarning(std::string_view message) { Log(Level::kWarning, message); }

void LogError(std::string_view message) { Log(Level::kError, message); }

void LogInfoToStdout(std::string_view message) {
  Log(Level::kInfo, message, true);
}

void Logf(Level level, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  const std::string formatted = FormatString(fmt, args);
  va_end(args);
  AppendEntry(level, formatted, false);
}

void LogfToStdout(Level level, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  const std::string formatted = FormatString(fmt, args);
  va_end(args);
  AppendEntry(level, formatted, true);
}

std::vector<Entry> Snapshot() {
  auto& state = GetState();
  std::scoped_lock lock(state.mutex);
  return std::vector<Entry>(state.entries.begin(), state.entries.end());
}

void Clear() {
  auto& state = GetState();
  std::scoped_lock lock(state.mutex);
  state.entries.clear();
}

bool ConsumeScrollRequest() {
  auto& state = GetState();
  std::scoped_lock lock(state.mutex);
  const bool should_scroll = state.scroll_to_bottom;
  state.scroll_to_bottom = false;
  return should_scroll;
}

}  // namespace fsai::sim::log
