#include "pid_controller.h"

namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki,
                             const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
  double pValue = error;
  double iValue = error * dt;
  double dValue = (error - previous_error_) / dt;
  double controlValue = kp_ * pValue + ki_ * iValue + kd_ * dValue;
  previous_error_ = error;
  return controlValue;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
  kp_ = 1;
  ki_ = 1;
  kd_ = 1;
}

}  // namespace control
}  // namespace shenlan