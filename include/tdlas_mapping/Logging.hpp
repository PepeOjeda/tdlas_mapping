#pragma once 

#include <rclcpp/logging.hpp>
#include <fmt/format.h>
#include <fmt/color.h>


#define TDLAS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("TDLAS"), fmt::format(__VA_ARGS__).c_str())
#define TDLAS_INFO_COLOR(color,...) RCLCPP_INFO(rclcpp::get_logger("TDLAS"), fmt::format(fmt::fg(color),__VA_ARGS__).c_str())

#define TDLAS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("TDLAS"), fmt::format(__VA_ARGS__).c_str())

#define TDLAS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("TDLAS"), fmt::format(__VA_ARGS__).c_str())

#define TDLAS_ASSERT(cnd)                                                                \
    {                                                                                    \
        if (!(cnd))                                                                      \
        {                                                                                \
            TDLAS_ERROR("Failed assertion at {}",                                       \
                      fmt::format(fmt::emphasis::bold, "{0}:{1}", __FILE__, __LINE__));  \
            raise(SIGTRAP);                                                              \
        }                                                                                \
    }
