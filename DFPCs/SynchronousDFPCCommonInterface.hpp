/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SynchornousDFPCCommonInterface.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco 
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This is the DFPC Common Interface which all Synchronous DFPCs should implement.
 *  A synchonous DFPC collects all the required input at some point in time before starting the processing of a chain of DFNs. 
 * 
 * @{
 */
#ifndef SYNCHRONOUS_DFPC_COMMON_INTERFACE_HPP
#define SYNCHRONOUS_DFPC_COMMON_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFPCCommonInterface.hpp>
#include <DFNsChainInterface>
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
    class SynchornousDFPCCommonInterface : public DFPCCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            SynchornousDFPCCommonInterface(DFNsChainInterface dfnsChain) : DFPCCommonInterface() 
		{
		this->dfnsChain = dfnsChain;
		}
            virtual ~SynchornousDFPCCommonInterface() {}

            final void run()
		 {
		 while ( dfpcIsActive() )
			{
		 	acquireInputs(); 
		 	dfnsChain->process(); 
		 	publishOutputs();
			} 
		 }
            final void setup() 
		{ 
		dfnsChain.setConfigurationFile(configurationFilePath);
		dfnsChain.configure(); 
		configureCommunication(); 
		}

            // DISCUSS: loggingIsActivated is an input port for each DFN.
            void loggingIsActivatedInput(LogLevel data)
            {
		DFPCCommonInterface::loggingIsActivatedInput(data);
                dfnsChain.loggingIsActivatedInput(data);
            }

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            DFNsChainIterface dfnsChain;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		virtual void configureCommunication() = 0;
		virtual void acquireInputs() = 0;
		virtual void publishOutputs() = 0;
		virtual bool dfpcIsActive() = 0;
    };
}
#endif
/* SynchornousDFPCCommonInterface.h */
/** @} */
