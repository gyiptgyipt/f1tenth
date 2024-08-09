#include <iostream>
#include <cmath>

//check_pid

class PID {
public:
    PID(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : kp(kp), ki(ki), kd(kd), _integral(0.0), _prev_error(0.0), _prev_derivative(0.0), _first_run(true) {}

    double update(double error, double dt = 0.01) {
        if (_first_run) {
            _prev_error = error;
            _first_run = false;
        }

        double derivative = (error - _prev_error) / dt;
        _integral += error * dt;
        _prev_error = error;

        double adjusted_kp = kp;
        double error_dampening_threshold = 0.05;
        if (std::abs(error) < error_dampening_threshold) {
            adjusted_kp = std::abs(error) * (1.0 / error_dampening_threshold) * kp;
        }

        // Uncomment if you want to apply a low-pass filter to the derivative term
        // if (_first_run) {
        //     _prev_derivative = derivative;
        // }
        // derivative = 0.6 * derivative + 0.4 * _prev_derivative;
        // _prev_derivative = derivative;

        double adjustment = error * adjusted_kp + derivative * kd + _integral * ki;

        // Prevent integral windup
        if (std::abs(adjustment) >= 0.1) {
            _integral = 0.0;
        }

        return adjustment;
    }

private:
    double kp, ki, kd;
    double _integral;
    double _prev_error;
    double _prev_derivative;
    bool _first_run;
};

// int main() {
//     PID pid(1.0, 0.1, 0.01);
//     double error = 0.5;
//     double dt = 0.01;
//     double adjustment = pid.update(error, dt);
//     std::cout << "Adjustment: " << adjustment << std::endl;

//     return 0;
// }