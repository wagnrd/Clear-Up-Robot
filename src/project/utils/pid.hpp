#pragma once

#include "utils.hpp"

namespace Utils
{
template <typename T>
class PidController
{
  /*
   * NOTE FOR MYSELF: 
   * should error_sum be clamped? 
   * Imagine the following Scenario: the camera should be controlled. it detects something on the bottom side of the screen.
   * The PID will now respond and move the camera down. But at some point, it cant go further down. If we are still tracking 
   * something at the bottom of the screen, the error will always be there and always have the same direction (move cam down).
   * That means, error_sum will accumulate constantly and will possibly overflow or block the camera in a down looking
   * position even if she tracks something further up, until the accumulated error has fallen to 0.
   * 
   *
   * the formulas have been verified with differend sources in the internet, but the implementation is written completly by myself
   * 
   * AUTHOR: Jonas Eckstein
   */

public:
  PidController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd)
  {
  }

  T update(T error, double deltaT = 1)
  {

    //calculate proportional component
    T p = kp * error;

    //calculate integral component
    error_sum += error;
    T i = ki * error_sum * deltaT;

    //calculate differential component
    T d = kd * (error - error_old) / deltaT;

    //add up components
    T pid = p + i + d;

    //save current error for next time
    error_old = error;

    return pid;
  }

  void reset()
  {
    error_sum = T();
    error_old = T();
  }

private:
  double kp, ki, kd;
  T error_sum;
  T error_old;
  bool clamped;
};
} // namespace Utils