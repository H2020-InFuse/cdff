/**
 * @author Alexander Fabisch 
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DFN_COMMON_INTERFACE_HPP
#define DFN_COMMON_INTERFACE_HPP

#include <stdint.h>
#include <stdlib.h>
#include <string>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN Common Interface: interface that every DFN must implement
     */
    class DFNCommonInterface
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

            DFNCommonInterface() : outputUpdated(true), executionTime(0), logLevel(OFF) {}
            virtual ~DFNCommonInterface() {}
            virtual void process() = 0;
            virtual void configure() = 0;

            // DISCUSS: This is a way to communicate if the output has been
            //          updated or not. If the DFN developer decides to not
            //          change the member 'resultUpdated', it is assumed that
            //          you can always request the current output values.
            //          Why does it make sense? An example: a node that fuses
            //          several laser scans to a point cloud might only
            //          generate a full point cloud from several accumulated
            //          laser scans, i.e., process() will be called multiple
            //          times before you can request the output from the DFN.
            //          An alternative would be to have one flag per port.
            virtual bool hasNewOutput()
            {
                return outputUpdated;
            }

            // DISCUSS: executionTime is an input port for each DFN.
            virtual void executionTimeInput(int64_t data)
            {
                executionTime = data;
            }

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

            bool outputUpdated;
            int64_t executionTime;
            LogLevel logLevel;
            std::string configurationFilePath;
    };
}
}

#endif // DFN_COMMON_INTERFACE_HPP

/** @} */
