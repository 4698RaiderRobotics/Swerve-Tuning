// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "DataLogger.h"

void Robot::RobotInit() {
  LoggedRobot::RobotInit();

}

void Robot::RobotPeriodic() {
  LoggedRobot::RobotPeriodic();

  m_drive.Periodic();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  frc::ChassisSpeeds speeds; 

  speeds.vx = vx_axis.GetAxis() * physical::kMaxDriveSpeed;
  speeds.vy = vy_axis.GetAxis() * physical::kMaxDriveSpeed;
  speeds.omega = omega_axis.GetAxis() * physical::kMaxTurnSpeed;
  m_drive.Drive( speeds );
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
