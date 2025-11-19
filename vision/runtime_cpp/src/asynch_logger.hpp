/**
 * @brief Asynchronous logger
 */

#pragma once
#include <string>
#include <quill/Frontend.h>
#include <quill/LogMacros.h>
#include <quill/sinks/FileSink.h>
#include <quill/Backend.h>

namespace fsai::vision {
    class Asynch_logger{
    public:
        /**
         * @brief Constructs a simple async logger.
         * @param logger_name Name of the logger.
         * @param file_path Path to the output log file.
         */
        explicit Asynch_logger(const std::string& logger_name = "app_logger", const std::string& file_path   = "app.log");

        /**
         * @brief Logs an informational message.
         * @param msg string.
         */
	void info(const std::string& msg);

        /**
         * @brief Logs a warning message.
         * @param msg string.
         */
	void warn(const std::string& msg);

        /**
         * @brief Logs an error message.
         * @param msg string.
         */
	void error(const std::string& msg);

        /**
         * @brief Logs a debug message.
         * @param msg string.
         */
	void debug(const std::string& msg);

    private:
        quill::Logger* logger_;
    };
}
