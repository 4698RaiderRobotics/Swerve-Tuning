// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "LoggedRobot.h"

#include "ControllerAxis.h"
#include "SwerveDrive.h"

class Robot : public LoggedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::XboxController m_xbox{0};
  ControllerAxis vx_axis{m_xbox, frc::XboxController::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_xbox, frc::XboxController::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_xbox, frc::XboxController::Axis::kRightX, true};

  SwerveDrive m_drive;

  frc::Pose2d m_move_delta;
  frc::Rotation2d m_finishHeading;
  units::second_t m_trajStart{ 0_s };

  frc::Trajectory m_trajectory;
  frc::TrajectoryConfig m_config{ 2_mps, 2_mps_sq };

};
