/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DfpcConfigurator.hpp
 * @date 21/03/2018
 * @author Alessandro Bianco 
 */

/*!
 * @addtogroup DFPCs
 * 
 * @brief This class handles the common configuration operation for each DFPC. 
 *
 * The DfpcConfigurator adds the following capability:
 * (i) automatic splitting of DFPC configuration file in multiple configuration files, one for each DFN so that each DFN can access its proper configuration file;
 * (ii) instantiation of each DFN according to the information written in the DFNsChain configuration file
 *
 * With the above functionality, a DFPC will need to
 * acquire access to the instantiated DFNs;
 * acquire extra DFPC configuration parameters beyond those used by the DFNs.
 * 
 * @{
 */
#ifndef DFPC_CONFIGURATOR_HPP
#define DFPC_CONFIGURATOR_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <map>
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
    class DfpcConfigurator
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:

            DfpcConfigurator();
            ~DfpcConfigurator();

            void configure(std::string configurationFilePath);
	    std::string GetExtraParametersConfigurationFilePath();
	    dfn_ci::DFNCommonInterface* GetDfn(std::string dfnName, bool optional = false);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
	    std::string extraParametersConfigurationFilePath;

	    std::map<std::string, dfn_ci::DFNCommonInterface*> dfnsSet;
	    std::map<std::string, std::string> configurationFilesSet;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		void ConstructDFNs(YAML::Node configuration);
		void SplitConfigurationFile(YAML::Node configuration,  std::string folderPath);
		std::string ComputeConfigurationFolderPath(std::string configurationFilePath);
		void ConfigureDfns();
		void DestroyDfns();
    };
}
#endif
/* DfpcConfigurator.h */
/** @} */
