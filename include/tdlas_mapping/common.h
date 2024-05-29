#pragma once
#include <tf2/LinearMath/Vector3.h>
#include <rclcpp/rclcpp.hpp>
#include <json/json.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/color_rgba.hpp>

static float signedAngle(const tf2::Vector3& v1, const tf2::Vector3& v2, const tf2::Vector3& referenceAxis)
{
    float sign = (v1.cross(v2).dot(referenceAxis)) > 0 ? 1 : -1;
    return v1.angle(v2) * sign;
}

static float signedDistanceToLine(const tf2::Vector3& lineOrigin, const tf2::Vector3& lineDirection, const tf2::Vector3& p)
{
    tf2::Vector3 vecToP = p - lineOrigin;
    tf2::Vector3 projected = lineOrigin + lineDirection.normalized() * lineDirection.normalized().dot(vecToP);

    float sign = signedAngle(lineDirection, vecToP, {0, 0, 1}) > 0 ? 1 : -1;

    return tf2::tf2Distance2(p, projected) * sign;
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

#include <glm/vec2.hpp>
namespace glm
{
    static glm::vec2 fromTF(const tf2::Vector3& v)
    {
        return glm::vec2(v.x(), v.y());
    }

    static geometry_msgs::msg::Point toPoint (const glm::vec2& vec)
    {
        geometry_msgs::msg::Point p;
        p.x = vec.x;
        p.y = vec.y;
        return p;
    }
}

class RollingAverage
{
public:
    RollingAverage() : numValues(0), currentAverage(0.0f)
    {}
    float Update(float next)
    {
        currentAverage = (currentAverage * numValues + next) / (numValues + 1);
        numValues++;
        return currentAverage;
    }

    float Get()
    {
        return currentAverage;
    }

    operator float()
    {
        return Get();
    }

private:
    uint numValues;
    float currentAverage;
};

inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = std::clamp(r, 0.0f, 1.0f);
    color.g = std::clamp(g, 0.0f, 1.0f);
    color.b = std::clamp(b, 0.0f, 1.0f);
    color.a = std::clamp(a, 0.0f, 1.0f);
    return color;
}

inline double lerp(double start, double end, double proportion)
{
    if (proportion < 0 || std::isnan(proportion))
        return start;

    return start + (end - start) * std::min(1.0, proportion);
}

enum valueColorMode
{
    Linear,
    Logarithmic
};
inline std_msgs::msg::ColorRGBA valueToColor(double val, double lowLimit, double highLimit, valueColorMode mode)
{
    double r, g, b;
    double range;
    if (mode == valueColorMode::Logarithmic)
    {
        val = std::log10(val);
        range = (std::log10(highLimit) - std::log10(lowLimit)) / 4;
        lowLimit = std::log10(lowLimit);
    }
    else
    {
        range = (highLimit - lowLimit) / 4;
    }

    if (val < lowLimit + range)
    {
        r = 0;
        g = lerp(0, 1, (val - lowLimit) / (range));
        b = 1;
    }
    else if (val < lowLimit + 2 * range)
    {
        r = 0;
        g = 1;
        b = lerp(1, 0, (val - (lowLimit + range)) / (range));
    }
    else if (val < lowLimit + 3 * range)
    {
        r = (val - (lowLimit + 2 * range)) / (range);
        g = 1;
        b = 0;
    }
    else
    {
        r = 1;
        g = lerp(1, 0, (val - (lowLimit + 3 * range)) / (range));
        b = 0;
    }
    return makeColor(r, g, b, 1);
}

#include <random>
inline float RandomFromGaussian(double mean, double stdev)
{
    static thread_local std::minstd_rand0 RNGengine;
    static thread_local std::normal_distribution<> dist{0, stdev};
    static thread_local double previousStdev = stdev;

    if (stdev != previousStdev)
    {
        dist = std::normal_distribution<>{0, stdev};
        previousStdev = stdev;
    }

    return mean + dist(RNGengine);
}