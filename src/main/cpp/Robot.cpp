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

  frc::SmartDashboard::PutNumber("Move Delta X", 0.5 );
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
  if( m_useTrajectory != frc::SmartDashboard::GetBoolean("Use Trajectory", false ) ) {
    m_useTrajectory = frc::SmartDashboard::GetBoolean("Use Trajectory", false );
    m_move_delta = {0.0_m, 0.0_m, 0_deg};  // Force regen of move
  };

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
    if( m_moveStart < 20_ms ) {
      m_moveStart = frc::Timer::GetFPGATimestamp();
      m_drive.ResetPose( {0_m, 0_m} );
    }
  }

  if( m_moveStart > 20_ms ) {
    units::second_t elapsed = frc::Timer::GetFPGATimestamp() - m_moveStart;
    if( elapsed > m_moveLength ) {
        // Finished trajectory
      m_moveStart = 0_s;
      m_moveLength = 0_s;
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
    m_drive.ArcadeDrive( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis(), false );
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::ConstructTrajectory() {
  frc::Pose2d startPose{ 0_m,0_m,0_deg};
  // frc::Pose2d startPose = m_drive.GetPose();
  // frc::Pose2d endPose{ startPose.X() + m_move_delta.X(),
  //                      startPose.Y() + m_move_delta.Y(),
  //                      startPose.Rotation() + m_move_delta.Rotation() };
  m_finishHeading = startPose.Rotation() + m_move_delta.Rotation();
  frc::SmartDashboard::PutNumber("End Heading", m_finishHeading.Degrees().value() );

  auto linear_tangent =  frc::Rotation2d{ m_move_delta.X().value(), m_move_delta.Y().value()};

  frc::Pose2d midPose{ startPose.X() + m_move_delta.X()/2.0,
                       startPose.Y() + m_move_delta.Y()/2.0,
                       startPose.Rotation() + m_move_delta.Rotation() };

  fmt::print( "\n\n ======= Trajectory =======\n" ); 
  fmt::print( "  Move Delta: {:9.4}, {:9.4}, Heading {:9.4}\n", 
              m_move_delta.X(), m_move_delta.Y(), m_move_delta.Rotation().Degrees() );
  // fmt::print( "  Start Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
  //             startPose.X(), startPose.Y(), startPose.Rotation().Degrees() );
  // fmt::print( "    End Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
  //             endPose.X(), endPose.Y(), endPose.Rotation().Degrees() );

  m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    {startPose.X(), startPose.Y(), linear_tangent}, 
    {midPose.X(), midPose.Y(), linear_tangent},
    {m_move_delta.X(), m_move_delta.Y(), linear_tangent}},
    m_config );

  m_moveLength = m_trajectory.TotalTime();
  fmt::print("Trajectory time: {}\n\n\n", m_moveLength );
}

void Robot::ConstructProfiles() {
  frc::Pose2d startPose{ 0_m,0_m,0_deg};
  // frc::Pose2d startPose = m_drive.GetPose();
  // frc::Pose2d endPose{ startPose.X() + m_move_delta.X(),
  //                      startPose.Y() + m_move_delta.Y(),
  //                      startPose.Rotation() + m_move_delta.Rotation() };

  fmt::print( "\n\n ======= Profile =======\n" ); 
  fmt::print( "  Move Delta: {:9.4}, {:9.4}, Heading {:9.4}\n", 
              m_move_delta.X(), m_move_delta.Y(), m_move_delta.Rotation().Degrees() );
  // fmt::print( "  Start Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
  //             startPose.X(), startPose.Y(), startPose.Rotation().Degrees() );
  // fmt::print( "    End Pose: {:9.4}, {:9.4}, Heading {:9.4}\n", 
  //             endPose.X(), endPose.Y(), endPose.Rotation().Degrees() );

  m_xProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, {m_move_delta.X()}, {startPose.X()} };
  m_yProfile = frc::TrapezoidProfile<units::meters> { m_linearConstraints, {m_move_delta.Y()}, {startPose.Y()} };
  m_omegaProfile = frc::TrapezoidProfile<units::degrees> { m_omegaConstraints, {m_move_delta.Rotation().Degrees()}, 
                                                          {startPose.Rotation().Degrees()} };

  if( m_xProfile.TotalTime() > m_yProfile.TotalTime() ) {
    m_moveLength = m_xProfile.TotalTime();
  } else {
    m_moveLength = m_yProfile.TotalTime();
  }
  if( m_omegaProfile.TotalTime() > m_moveLength ) {
    m_moveLength = m_omegaProfile.TotalTime();
  }

  fmt::print("Profile time: {}\n\n\n", m_moveLength );
}

void Robot::ProfiledMove( units::second_t elapsed ) {
  frc::TrapezoidProfile<units::meters>::State xSetpoint = m_xProfile.Calculate( elapsed );
  frc::TrapezoidProfile<units::meters>::State ySetpoint = m_yProfile.Calculate( elapsed );
  frc::TrapezoidProfile<units::degrees>::State omegaSetpoint = m_omegaProfile.Calculate( elapsed );

  frc::SmartDashboard::PutNumber("Profiled X Setpoint", xSetpoint.position.value());
  frc::SmartDashboard::PutNumber("Profiled Y Setpoint", ySetpoint.position.value());
  frc::SmartDashboard::PutNumber("Profiled Theta Setpoint", omegaSetpoint.position.value());

  frc::SmartDashboard::PutNumber("Profiled X Velocity Setpoint", xSetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("Profiled Y Velocity Setpoint", ySetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("Profiled Omega Setpoint", omegaSetpoint.velocity.value());

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
