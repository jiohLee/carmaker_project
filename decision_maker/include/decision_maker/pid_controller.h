#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <iostream>
#include <deque>

class PIDController
{
public:
    PIDController(float Kp = 0, float Ki = 0, float Kd = 0, size_t qSize = 0);

    float getControlInput(float target, float current, float dt);
    float getError() {return error; }

    void setGains(float Kp, float Ki, float Kd) { Kp_ = Kp; Ki_ = Ki; Kd_ = Kd; }
    void setQueueSize(size_t qSize) { qSize_ = qSize; }
private:

    size_t qSize_ = 0;

    float Kp_ = 0;
    float Ki_ = 0;
    float Kd_ = 0;

    float error = 0;
    float errorPrev = 0;

    std::deque<float> errorQ;

    float derivative;
    float integral;
};

#endif // PIDCONTROLLER_H
