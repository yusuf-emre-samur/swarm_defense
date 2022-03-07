#ifndef SD_PID_HPP_
#define SD_PID_HPP_

namespace sd {

class PID
{
  public:
	PID();

  private:
	double kp_, ki_, kd_;
};

} // namespace sd

#endif