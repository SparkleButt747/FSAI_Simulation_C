#include "asynch_logger.hpp"
#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/sinks/FileSink.h>
#include <quill/LogMacros.h>

namespace fsai::vision
{
    Asynch_logger::Asynch_logger(const std::string& logger_name, const std::string& file_path)
    {
        quill::Backend::start();
        auto sink = std::make_shared<quill::FileSink>(file_path.c_str());
        logger_ = quill::Frontend::create_or_get_logger( logger_name.c_str(), { sink } );
    }

    void Asynch_logger::info(const std::string& msg)
    {
        LOG_INFO(logger_, "{}", msg);
    }

    void Asynch_logger::warn(const std::string& msg)
    {
        LOG_WARNING(logger_, "{}", msg);
    }

    void Asynch_logger::error(const std::string& msg)
    {
        LOG_ERROR(logger_, "{}", msg);
    }

    void Asynch_logger::debug(const std::string& msg)
    {
        LOG_DEBUG(logger_, "{}", msg);
    }
}
