#pragma once

#include <iostream>
#include <memory>
#include <string_view>

namespace velox::logging {

enum class Level {
    Debug,
    Info,
    Warning,
    Error,
};

class LogSink {
public:
    virtual ~LogSink() = default;
    virtual void log(Level level, std::string_view message) = 0;
};

inline void log(LogSink* sink, Level level, std::string_view message)
{
    if (sink) {
        sink->log(level, message);
    }
}

class OstreamLogSink : public LogSink {
public:
    explicit OstreamLogSink(std::ostream& os)
        : os_(os)
    {
    }

    void log(Level level, std::string_view message) override
    {
        os_ << prefix(level) << message << std::endl;
    }

private:
    std::ostream& os_;

    static constexpr const char* prefix(Level level)
    {
        switch (level) {
            case Level::Debug: return "[debug] ";
            case Level::Info: return "[info ] ";
            case Level::Warning: return "[warn ] ";
            case Level::Error: return "[error] ";
        }
        return "";
    }
};

using LogSinkPtr = std::shared_ptr<LogSink>;

inline LogSinkPtr make_console_log_sink()
{
    return std::make_shared<OstreamLogSink>(std::clog);
}

} // namespace velox::logging

