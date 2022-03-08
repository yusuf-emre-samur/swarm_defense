#ifndef SD_PID_CONTROLLER_HPP_
#define SD_PID_CONTROLLER_HPP_

class PID
{
  public:
	PID(const double& dt, const double& min, const double& max,
		const double& kp, const double& kd, const double& ki);
	~PID() = default;
	double calculate(const double& reference, const double& measurement);

  private:
	double dt_;
	double max_;
	double min_;
	double kp_;
	double kd_;
	double ki_;
	double pre_error_;
	double integral_;
};
#endif