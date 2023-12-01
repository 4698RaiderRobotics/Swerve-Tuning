// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fmt/format.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Robot.h"

#include "DataLogger.h"

void Robot::RobotInit() {
  LoggedRobot::RobotInit();

  m_drive.StartLogging( frc::DataLogManager::GetLog() );

  frc::SmartDashboard::PutNumber("Move Delta X", 0.0 );
  frc::SmartDashboard::PutNumber("Move Delta Y", 0.0 );
  frc::SmartDashboard::PutNumber("Move Angle", 0.0 );

}

void Robot::RobotPeriodic() {
  LoggedRobot::RobotPeriodic();

  m_drive.Periodic();

  frc::Pose2d dashboardPose;

  dashboardPose.X() = units::meter_t{ frc::SmartDashboard::GetNumber("Move Delta X", 0.0 ) };
  dashboardPose.Y() = units::meter_t{ frc::SmartDashboard::PutNumber("Move Delta Y", 0.0 ) };
  dashboardPose.Rotation().Degrees() = units::degree_t{ frc::SmartDashboard::PutNumber("Move Angle", 0.0 ) };

  if( m_move_delta != dashboardPose ) {
    m_move_delta = dashboardPose;
    if( m_move_delta.Translation().Norm() > 0.001_m ) {
      frc::Pose2d startPose = m_drive.GetPose();
      frc::Pose2d endPose;
      endPose = startPose + frc::Transform2d{ m_move_delta.Translation(), m_move_delta.Rotation() };
      m_finishHeading = startPose.Rotation() + m_move_delta.Rotation();

      auto linear_tangent =  frc::Rotation2d{ m_move_delta.X().value(), m_move_delta.Y().value()};

      frc::Pose2d midPose = startPose + frc::Transform2d{ m_move_delta.Translation() / 2.0, m_move_delta.Rotation() };;

      fmt::print( "   Start Pose: {}, {}, Rotation {}\n", startPose.X(), startPose.Y(), linear_tangent.Degrees() );
      fmt::print( "   End Pose {}, {}\n", endPose.X(), endPose.Y() );

    
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
        {startPose.X(), startPose.Y(), linear_tangent}, 
        {midPose.X(), midPose.Y(), linear_tangent},
        {endPose.X(), endPose.Y(), linear_tangent}},
        m_config );

      fmt::print("Trajectory time: {}\n", m_trajectory.TotalTime());

    }
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  if( m_xbox.GetAButtonPressed() ) {
    if( m_trajStart < 20_ms ) {
      m_trajStart = frc::Timer::GetFPGATimestamp();
    }
  }

  if( m_trajStart > 20_ms ) {
    units::second_t elapsed = frc::Timer::GetFPGATimestamp() - m_trajStart;
    if( elapsed > m_trajectory.TotalTime() ) {
        // Finished trajectory
      m_trajStart = 0_s;
      m_move_delta = {0.0_m, 0.0_m, 0_deg};  // Force regen of trajectory
    } else {
      auto goal = m_trajectory.Sample( elapsed );
      m_drive.DriveTrajectory( goal, m_finishHeading );
    }
  } else {
    m_drive.ArcadeDrive( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis() );
  }
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
