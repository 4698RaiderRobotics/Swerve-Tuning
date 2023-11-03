// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <wpi/DataLog.h>

class DataLogger {
    public:
        void StartDataLog( wpi::log::DataLog &d ) { log = &d; }

        void Send( std::string_view s, double val ) { 
            wpi::log::DoubleLogEntry d{ *(log), s };
            d.Append( val );
        }
    
    private:
        wpi::log::DataLog *log;
};