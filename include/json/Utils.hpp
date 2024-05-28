#pragma once
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>
#include "json.hpp"

static constexpr double Deg2Rad = (2 * M_PI) / 360;

namespace jsonSerialization
{

    namespace Utils
    {

        inline double getYaw(const geometry_msgs::msg::Quaternion& quat)
        {
            tf2::Quaternion tfquat;
            tf2::fromMsg(quat, tfquat);

            tf2::Matrix3x3 m(tfquat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }

        inline std::string applyNamespaceIfNeeded(std::string& topicName, rclcpp::Node::SharedPtr node)
        {
            if (topicName.at(0) != '/')
            {
                std::string _namespace = node->get_namespace();

                // handle the empty namespace
                if (_namespace == "/")
                    _namespace = "";

                return _namespace + "/" + topicName;
            }
            return topicName;
        }

        inline olfaction_msgs::msg::TDLAS jsonToTDLAS(const nlohmann::json& json)
        {
            olfaction_msgs::msg::TDLAS tdlas;
            tdlas.average_ppmxm = (double)json["average_ppmxm"].get<int>();
            tdlas.average_absorption_strength = json["average_absorption_strength"].get<double>();
            tdlas.average_reflection_strength = json["average_reflection_strength"].get<double>();
            return tdlas;
        }

        inline tf2::Transform parse_status_publisher_msg(const std::string& msg)
        {
            auto json = nlohmann::json::parse(msg);
            std::string x_y_yaw_string = json["data"]["pose"].get<std::string>();

            tf2::Transform tf;
            // parse the string
            std::array<double, 3> x_y_yaw;
            std::stringstream s_stream(x_y_yaw_string);
            int i = 0;

            s_stream.ignore(2, '[');
            while (!s_stream.eof())
            {
                // for debugging
                // std::string remaining = (s_stream.str().substr(s_stream.tellg()));

                s_stream >> x_y_yaw[i];
                s_stream.ignore(2, ' ');

                if (s_stream.fail())
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR PARSING THE JSON STRING: %s, \n\nPOSE SUBSTRING: %s:", msg.c_str(),
                                 x_y_yaw_string.c_str());
                i++;
            }

            tf.setOrigin({x_y_yaw[0], x_y_yaw[1], 0});
            tf.setRotation(tf2::Quaternion({0, 0, 1}, Deg2Rad * x_y_yaw[2]));

            return tf;
        }
    } // namespace Utils
} // namespace jsonSerialization