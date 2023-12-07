// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <units/angular_acceleration.h>

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

  void ProfiledMove( units::second_t );
  void ConstructTrajectory();
  void ConstructProfiles();

 private:
  frc::XboxController m_xbox{1};
  ControllerAxis vx_axis{m_xbox, frc::XboxController::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_xbox, frc::XboxController::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_xbox, frc::XboxController::Axis::kRightX, true};

  SwerveDrive m_drive;

  frc::Pose2d m_move_delta;
  frc::Rotation2d m_finishHeading;
  units::second_t m_moveStart{ 0_s };
  units::second_t m_moveLength{ 0_s };

  frc::Trajectory m_trajectory;
  frc::TrajectoryConfig m_config{ 2_mps, 2_mps_sq };

  frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{ 3_mps, 1.5_mps_sq };
  frc::TrapezoidProfile<units::degrees>::Constraints m_omegaConstraints{ 360_deg_per_s, 180_deg_per_s_sq };
  
  frc::TrapezoidProfile<units::meters> m_xProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::meters> m_yProfile{ m_linearConstraints, {0.0_m}, {0.0_m} };
  frc::TrapezoidProfile<units::degrees> m_omegaProfile{ m_omegaConstraints, {0.0_deg}, {0.0_deg} };

  frc::TrapezoidProfile<units::meters>::State xSetpoint;
  frc::TrapezoidProfile<units::meters>::State ySetpoint;
  frc::TrapezoidProfile<units::degrees>::State omegaSetpoint;

  bool m_useTrajectory{false};
};
