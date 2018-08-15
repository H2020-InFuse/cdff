/**
 * @author Alessandro Bianco 
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef DFPC_COMMON_INTERFACE_HPP
#define DFPC_COMMON_INTERFACE_HPP

#include <stdint.h>
#include <stdlib.h>
#include <string>

namespace dfpc_ci
{
    /**
     * DFPC Common Interface: interface that every DFPC must implement
     */
    class DFPCCommonInterface
    {
        public:

            enum LogLevel
            {
                OFF,
                ERROR,
                WARNING,
                INFO,
                DEBUG
            };

            DFPCCommonInterface() {}
            virtual ~DFPCCommonInterface() {}
            virtual void run() = 0;
            virtual void setup() = 0;

            // DISCUSS: loggingIsActivated is an input port for each DFN.
            virtual void loggingIsActivatedInput(LogLevel data)
            {
                logLevel = data;
            }

            virtual void setConfigurationFile(std::string configurationFilePath)
            {
                this->configurationFilePath = configurationFilePath;
            }


        protected:

            LogLevel logLevel;
            std::string configurationFilePath;
    };
}

#endif // DFPC_COMMON_INTERFACE_HPP

/** @} */
