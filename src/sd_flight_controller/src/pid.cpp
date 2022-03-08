#include "sd_flight_controller/pid.hpp"

PID::PID(const double& dt, const double& min, const double& max,
		 const double& kp, const double& kd, const double& ki)
	: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0),
	  integral_(0)
{
}

double PID::calculate(const double& reference, const double& measurement)
{
	// Calculate error
	const double error = reference - measurement;
	// Integral
	integral_ += error * dt_;
	// Derivative
	const double derivative = (error - pre_error_) / dt_;

	// Calculate total output
	double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

	// Restrict to max/min
	if ( output > max_ )
		output = max_;
	if ( output < min_ )
		output = min_;

	// Save error to previous error
	pre_error_ = error;

	return output;
}
