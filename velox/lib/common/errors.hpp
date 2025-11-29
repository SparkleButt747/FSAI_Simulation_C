#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

#include "simulation/model_timing.hpp"

namespace velox::errors {

inline std::string append_context(std::string message, std::string_view context)
{
    if (!context.empty()) {
        message = std::string(context) + ": " + message;
    }
    return message;
}

inline std::string with_location(std::string message, const char* file, int line)
{
    return append_context(std::move(message), std::string("at ") + file + ":" + std::to_string(line));
}

inline std::string with_model(std::string message, simulation::ModelType model)
{
    return append_context(std::move(message), std::string("model=") + simulation::model_display_name(model));
}

class VeloxError : public std::runtime_error {
public:
    explicit VeloxError(std::string message, std::string context = {})
        : std::runtime_error(append_context(std::move(message), context))
        , context_(context.empty() ? std::string{} : std::string{context})
    {
    }

    const std::string& context() const noexcept { return context_; }

private:
    std::string context_{};
};

class ConfigError : public VeloxError {
public:
    using VeloxError::VeloxError;
};

class InputError : public VeloxError {
public:
    using VeloxError::VeloxError;
};

class SimulationError : public VeloxError {
public:
    using VeloxError::VeloxError;
};

} // namespace velox::errors

#define VELOX_LOC(msg) ::velox::errors::with_location((msg), __FILE__, __LINE__)
#define VELOX_MODEL(msg, model) ::velox::errors::with_location(::velox::errors::with_model((msg), (model)), __FILE__, __LINE__)
#define VELOX_CONTEXT(msg, ctx) ::velox::errors::with_location(::velox::errors::append_context((msg), (ctx)), __FILE__, __LINE__)

