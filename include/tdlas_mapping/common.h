#pragma once
#include <tf2/LinearMath/Vector3.h>
#include <rclcpp/rclcpp.hpp>
#include <json.hpp>


static constexpr double Deg2Rad = (2*M_PI)/360;

static float signedAngle(const tf2::Vector3& v1, const tf2::Vector3& v2, const tf2::Vector3& referenceAxis)
{
    float sign = (v1.cross(v2).dot(referenceAxis)) >0? 1 : -1;
    return v1.angle(v2) * sign;
}

static float signedDistanceToLine(const tf2::Vector3& lineOrigin, const tf2::Vector3& lineDirection, const tf2::Vector3& p )
{
    tf2::Vector3 vecToP = p-lineOrigin;
    tf2::Vector3 projected = lineOrigin + lineDirection.normalized() * lineDirection.normalized().dot(vecToP);
    
    float sign = signedAngle( lineDirection, vecToP, {0,0,1}) >0? 1:-1;

    return tf2::tf2Distance2(p, projected) * sign;
}

static tf2::Transform parse_status_publisher_msg(const std::string& msg)
{
    auto json =  nlohmann::json::parse(msg); 
    std::string x_y_yaw_string = json["data"]["pose"].get<std::string>();
        
    tf2::Transform tf;
    //parse the string
    std::array<double, 3> x_y_yaw;
    std::stringstream s_stream(x_y_yaw_string);
    int i = 0;
    
    s_stream.ignore(2,'[');
    while(!s_stream.eof())
    {
        //for debugging
        //std::string remaining = (s_stream.str().substr(s_stream.tellg()));
        
        s_stream >> x_y_yaw[i];
        s_stream.ignore(2,' ');
        
        if(s_stream.fail())
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR PARSING THE JSON STRING: %s, \n\nPOSE SUBSTRING: %s:", msg.c_str(), x_y_yaw_string.c_str());
        i++;   
    }

    tf.setOrigin( {x_y_yaw[0], x_y_yaw[1], 0} );
    tf.setRotation( tf2::Quaternion({0,0,1}, Deg2Rad * x_y_yaw[2]) );
    
    return tf;
}

static geometry_msgs::msg::Point VecToPoint(const tf2::Vector3& vec)
{
    geometry_msgs::msg::Point p;
    p.x = vec.x();
    p.y = vec.y();
    p.z = vec.z();
    
    return p;
}

static geometry_msgs::msg::Pose transformToPose(const tf2::Transform& transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position = VecToPoint(transform.getOrigin());
    pose.orientation = tf2::toMsg(transform.getRotation());
    return pose;
}


static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose& pose)
{
    tf2::Transform transform;
    tf2::Vector3 vec;
    vec.setX(pose.position.x);
    vec.setY(pose.position.y);
    vec.setZ(pose.position.z);
    transform.setOrigin(vec);

    tf2::Quaternion quat;
    tf2::fromMsg(pose.orientation, quat);
    transform.setRotation(quat);
    return transform;
}
