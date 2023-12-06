//
//      Base Class for a logged robot to write basic data to a log file.
//

#include <frc/RobotController.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>

#include "DataLogger.h"
#include "LoggedRobot.h"

void LoggedRobot::RobotInit() {
        // Disable LiveWindow Telemetry
    frc::LiveWindow::DisableAllTelemetry();
    
        // Start the log manager
    frc::DataLogManager::Start();

        // Record both DS control and joystick data
    frc::DriverStation::StartDataLog( frc::DataLogManager::GetLog() );

        // Send the metadata from the buildinfo.txt file
    DataLogger::GetInstance().LogMetadata();

        // Determine the number of PDP channels
    m_pdpChannels = (m_pdp.GetType() == frc::PowerDistribution::ModuleType::kRev) ? 24 : 16;
}

void LoggedRobot::RobotPeriodic() {
    static double currents[24];
    frc::PowerDistribution::Faults faults;

    DataLogger &logger = DataLogger::GetInstance();

    frc::CANStatus cs;

        // Log the Robot Controller Information
    logger.Send( "RobotCTL/Input Voltage", frc::RobotController::GetInputVoltage() );
    logger.Send( "RobotCTL/Input Current", frc::RobotController::GetInputCurrent() );
    logger.Send( "RobotCTL/BrownedOut", frc::RobotController::IsBrownedOut() );
    logger.Send( "RobotCTL/3V3 Volts", frc::RobotController::GetVoltage3V3() );
    logger.Send( "RobotCTL/3V3 Amps", frc::RobotController::GetCurrent3V3() );
    logger.Send( "RobotCTL/3V3 Fault Count", frc::RobotController::GetFaultCount3V3() );
    logger.Send( "RobotCTL/5V Volts", frc::RobotController::GetVoltage5V() );
    logger.Send( "RobotCTL/5V Amps", frc::RobotController::GetCurrent5V() );
    logger.Send( "RobotCTL/5V Fault Count", frc::RobotController::GetFaultCount5V() );
    cs = frc::RobotController::GetCANStatus();
    logger.Send( "RobotCTL/CAN Percent Utilization", cs.percentBusUtilization );
    logger.Send( "RobotCTL/CAN OffCount", cs.busOffCount );
    logger.Send( "RobotCTL/CAN receiveErrorCount", cs.receiveErrorCount );
    logger.Send( "RobotCTL/CAN transmitErrorCount", cs.transmitErrorCount );
    logger.Send( "RobotCTL/CAN txFullCount", cs.txFullCount );

        // Log the PDP Information
    faults = m_pdp.GetFaults();

    logger.Send( "PDP/Bus Voltage", m_pdp.GetVoltage() );
    logger.Send( "PDP/Total Current", m_pdp.GetTotalCurrent() );
    logger.Send( "PDP/Temperature", m_pdp.GetTemperature() );
    logger.Send( "PDP/Total Power", m_pdp.GetTotalPower() );
    logger.Send( "PDP/Total Energy", m_pdp.GetTotalEnergy() );
    logger.Send( "PDP/Brown Out", (bool) faults.Brownout );
    logger.Send( "PDP/Can Warning", (bool) faults.CanWarning );
    logger.Send( "PDP/Hardware Fault", (bool) faults.HardwareFault );

    for( int i=0; i<m_pdpChannels; ++i ) {
        currents[i] = m_pdp.GetCurrent( i );
    }
    logger.Send( "PDP/Currents", std::span<double> ( currents, m_pdpChannels) );
}
