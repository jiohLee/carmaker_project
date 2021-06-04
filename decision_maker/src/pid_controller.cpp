#include "decision_maker/pid_controller.h"

PIDController::PIDController(float Kp, float Ki, float Kd, size_t qSize)
    :Kp_(Kp) ,Ki_(Ki) ,Kd_(Kd) ,qSize_(qSize)
{ }

float PIDController::getControlInput(float target, float current, float dt)
{
    error = target - current;

    errorQ.push_front(error);
    if(errorQ.size() > qSize_) errorQ.pop_back();

    integral = 0;
    for (size_t i = 0; i < errorQ.size(); i++)
    {
        integral += errorQ[i] * dt;
    }

    derivative = (error - errorPrev) / dt;
    errorPrev = error;

    return Kp_ * error + Ki_ * integral + Kd_ * derivative;
}
