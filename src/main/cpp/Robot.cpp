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

  frc::SmartDashboard::PutNumber("Move Delta X", 0.01 );
  frc::SmartDashboard::PutNumber("Move Delta Y", 0.0 );
  frc::SmartDashboard::PutNumber("Move Angle", 0.0 );
  frc::SmartDashboard::PutBoolean("Use Trajectory", m_useTrajectory );
  frc::SmartDashboard::PutNumber("End Heading", 0.0 );

}

void Robot::RobotPeriodic() {
  LoggedRobot::RobotPeriodic();

  m_drive.Periodic();

  units::meter_t dx = frc::SmartDashboard::GetNumber("Move Delta X", 0.0 ) * 1_m;
  units::meter_t dy = frc::SmartDashboard::GetNumber("Move Delta Y", 0.0 ) * 1_m;
  units::degree_t dtheta = frc::SmartDashboard::GetNumber("Move Angle", 0.0 ) * 1_deg;
  m_useTrajectory = frc::SmartDashboard::GetBoolean("Use Trajectory", false );

  frc::Pose2d dashboardPose{ dx, dy, dtheta };

  frc::SmartDashboard::PutNumber("Current Pose X", m_drive.GetPose().X().value() );
  frc::SmartDashboard::PutNumber("Current Pose Y", m_drive.GetPose().Y().value() );
  frc::SmartDashboard::PutNumber("Current Pose Heading", m_drive.GetPose().Rotation().Degrees().value() );


  if( m_move_delta != dashboardPose ) {
    m_move_delta = dashboardPose;
    if( m_move_delta.Translation().Norm() > 0.001_m ) {
      if( m_useTrajectory ) {
        ConstructTrajectory();
      } else {
        ConstructProfiles();
      }
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
      if( m_useTrajectory ) {
        m_trajLength = m_trajectory.TotalTime();
      } else {
        if( m_xProfile.TotalTime() > m_yProfile.TotalTime() ) {
          m_trajLength = m_xProfile.TotalTime();
        } else {
          m_trajLength = m_yProfile.TotalTime();
        }
        if( m_omegaProfile.TotalTime() > m_trajLength ) {
          m_trajLength = m_omegaProfile.TotalTime();
        }
      }
    }
  }

  if( m_trajStart > 20_ms ) {
    units::second_t elapsed = frc::Timer::GetFPGATimestamp() - m_trajStart;
    if( elapsed > m_trajLength ) {
        // Finished trajectory
      m_trajStart = 0_s;
      m_move_delta = {0.0_m, 0.0_m, 0_deg};  // Force regen of trajectory
    } else {
      if( m_useTrajectory ) {
          // Drive along trajectory.
        auto goal = m_trajectory.Sample( elapsed );
        frc::SmartDashboard::PutNumber("Trajectory X Setpoint", goal.pose.X().value());
        frc::SmartDashboard::PutNumber("Trajectory Y Setpoint", goal.pose.Y().value());
        frc::SmartDashboard::PutNumber("Trajectory Theta Setpoint", goal.pose.Rotation().Degrees().value());

        m_drive.DriveTrajectory( goal, m_finishHeading );
      } else {
          // Do profiled moves for X, Y, and theta.
          ProfiledMove( elapsed );
      }
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

void Robot::ConstructTrajectory() {
  frc::Pose2d startPose = m_drive.GetPose();
  frc::Pose2d endPose;
  endPose = startPose + frc::Transform2d{ m_move_delta.Translation(), m_move_delta.Rotation() };
  m_finishHeading = startPose.Rotation() + m_move_delta.Rotation();
  frc::SmartDashboard::PutNumber("End Heading", m_finishHeading.Degrees().value() );

  auto linear_tangent =  frc::Rotation2d{ m_move_delta.X().value(), m_move_delta.Y().value()};

  frc::Pose2d midPose = startPose + frc::Transform2d{ m_move_delta.Translation() / 2.0, m_move_delta.Rotation() };;

  fmt::print( "  Start Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
              startPose.X(), startPose.Y(), startPose.Rotation().Degrees() );
  fmt::print( "    End Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
              endPose.X(), endPose.Y(), endPose.Rotation().Degrees() );


  m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    {startPose.X(), startPose.Y(), linear_tangent}, 
    {midPose.X(), midPose.Y(), linear_tangent},
    {endPose.X(), endPose.Y(), linear_tangent}},
    m_config );

  fmt::print("Trajectory time: {}\n", m_trajectory.TotalTime());
}

void Robot::ConstructProfiles() {
  frc::Pose2d startPose = m_drive.GetPose();
  frc::Pose2d endPose;
  endPose = startPose + frc::Transform2d{ m_move_delta.Translation(), m_move_delta.Rotation() };

  m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, {endPose.X()}, {startPose.X()} };
  m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, {endPose.Y()}, {startPose.Y()} };
  m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, {endPose.Rotation().Degrees()}, 
                                                          {startPose.Rotation().Degrees()} };
}

void Robot::ProfiledMove( units::second_t elapsed ) {
  frc::TrapezoidProfile<units::meters>::State xSetpoint = m_xProfile.Calculate( elapsed );
  frc::TrapezoidProfile<units::meters>::State ySetpoint = m_yProfile.Calculate( elapsed );
  frc::TrapezoidProfile<units::degrees>::State omegaSetpoint = m_omegaProfile.Calculate( elapsed );

  frc::SmartDashboard::PutNumber("Trapezoid X Setpoint", xSetpoint.position.value());
  frc::SmartDashboard::PutNumber("Trapezoid Y Setpoint", ySetpoint.position.value());
  frc::SmartDashboard::PutNumber("Trapezoid Theta Setpoint", omegaSetpoint.position.value());

  frc::SmartDashboard::PutNumber("Trapezoid X Velocity Setpoint", xSetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("Trapezoid Y Velocity Setpoint", ySetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("Trapezoid Omega Setpoint", omegaSetpoint.velocity.value());

  frc::ChassisSpeeds speeds;
  speeds.vx = xSetpoint.velocity;
  speeds.vy = ySetpoint.velocity;
  speeds.omega = omegaSetpoint.velocity;

  m_drive.Drive( speeds );
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
