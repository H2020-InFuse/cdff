/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DFNsChainInterface.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco 
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This is the DFNs Chain Interface for all chains of DFNs that needs to be used in sequence by a DFPC. 
 * 
 * @{
 */
#ifndef DFNs_CHAIN_INTERFACE_HPP
#define DFNs_CHAIN_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <map>
#include <DFNCommonInterface.hpp>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class DFNsChainInterface
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

            DFNsChainInterface();
            ~DFNsChainInterface();

            virtual void process() = 0;
            void configure();

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
            virtual bool hasNewOutput();

            // DISCUSS: executionTime is an input port for each DFN.
            virtual void executionTimeInput(int64_t data);

            // DISCUSS: loggingIsActivated is an input port for each DFN.
            virtual void loggingIsActivatedInput(LogLevel data);
	
            virtual void setConfigurationFile(std::string configurationFilePath);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            bool outputUpdated;
            int64_t executionTime;
            LogLevel logLevel;
	    std::string configurationFilePath;

	    std::map<std::string, dfn_ci::DFNCommonInterface*> dfnsSet;
	    std::map<std::string, std::string> configurationFilesSet;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		void ConstructDFNs(YAML::Node configuration);
		void SplitConfigurationFile(YAML::Node configuration);
		std::string ComputeConfigurationFolderPath();
		void ConfigureDfns();
		void DestroyDFNs();
		virtual void AssignDfnsAlias();
    };
}
#endif
/* DFNsChainInterface.h */
/** @} */
