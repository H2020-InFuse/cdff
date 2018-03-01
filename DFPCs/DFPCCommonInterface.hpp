/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DFPCCommonInterface.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco 
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This is the DFPC Common Interface which all DFPCs should implement. 
 * 
 * @{
 */
#ifndef DFPC_COMMON_INTERFACE_HPP
#define DFPC_COMMON_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdlib.h>
#include <string>

namespace dfpc_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class DFPCCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            enum LogLevel
            {
                OFF,
                ERROR,
                WARNING,
                INFO,
                DEBUG
            };

            DFPCCommonInterface() : outputUpdated(true) {}
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

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            LogLevel logLevel;
	    std::string configurationFilePath;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

    };
}
#endif
/* DFPCCommonInterface.h */
/** @} */
