//
//      Base Class for a logged robot to write basic data to a log file.
//

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

class LoggedRobot : public frc::TimedRobot {
  public:
    void RobotInit() override;
    void RobotPeriodic() override;

  private:
    frc::PowerDistribution m_pdp{};
    int m_pdpChannels;
};