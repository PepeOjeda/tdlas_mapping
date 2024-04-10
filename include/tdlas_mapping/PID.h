#pragma once
#include <rclcpp/rclcpp.hpp>

class PID
{
public:
    PID() = delete;
    PID(rclcpp::Clock::SharedPtr _clock, float p, float i, float d) : clock(_clock), kP(p), kI(i), kD(d)
    {
        reset(0);
    }

    void reset(float error){
        setPrevious(error);
        integralAccumulator = 0;
    }

    float DoUpdate(float error){
        float timestep = clock->now().seconds()-previousTimestamp;
        
        float pTerm = error * kP;

        float dTerm = timestep==0? 0 : kD*(error - previousError)/timestep;
        setPrevious(error);

        integralAccumulator = std::clamp(integralAccumulator + error*timestep, -integralSaturation, integralSaturation);
        float iTerm = integralAccumulator * kI;
        
        return std::clamp(pTerm + dTerm + iTerm, -maximumOutput, maximumOutput);
    }

    float kP;
    float kD;
    float kI;
    float maximumOutput = 1;
private:
    rclcpp::Clock::SharedPtr clock;
    float integralSaturation;

    float previousError;
    float previousTimestamp;
    float integralAccumulator;

    void setPrevious(float val){
        previousError = val;
        previousTimestamp = clock->now().seconds();
    }

};