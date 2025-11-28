#include "asynch_logger.hpp"

namespace fsai::vision
{
    Asynch_logger::Asynch_logger(const std::string& logger_name, const std::string& file_path)
        : file_(file_path, std::ios::app), logger_name_(logger_name)
    {
    }

    void Asynch_logger::info(const std::string& msg)
    {
        log("INFO", msg);
    }

    void Asynch_logger::warn(const std::string& msg)
    {
        log("WARN", msg);
    }

    void Asynch_logger::error(const std::string& msg)
    {
        log("ERROR", msg);
    }

    void Asynch_logger::debug(const std::string& msg)
    {
        log("DEBUG", msg);
    }

    void Asynch_logger::log(const char* level, const std::string& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open())
        {
            file_ << "[" << level << "] " << msg << '\n';
            file_.flush();
        }
        std::cout << "[" << logger_name_ << "][" << level << "] " << msg << std::endl;
    }
}
