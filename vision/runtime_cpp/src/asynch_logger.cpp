#pragma once
#include "asynch_logger.hpp"

namespace fsai:vision
{
	Asynch_logger::Asynch_logger(const std::string& logger_name = "app_logger", const std::string& file_path = "app.log"):
	{
		quill::start();
		logger_ = quill::get_logger(logger_name.c_str());
		logger_->add_sink(quill::file_sink(file_path.c_str()));
	}

	Asynch_logger::info
	{
		LOG_INFO(logger_, "{}", msg);
	}

	Asynch_logger::warn
	{
		LOG_WARNING(logger_, "{}", msg);
	}

	Asynch_logger::error
	{
		LOG_ERROR(logger_, "{}", msg);
	}

	Asynch_logger::debug
	{
		LOG_DEBUG(logger_, "{}", msg);
	}
}

