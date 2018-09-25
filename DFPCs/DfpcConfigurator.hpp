/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFPCs
 * @{
 */
#ifndef DFPC_CONFIGURATOR_HPP
#define DFPC_CONFIGURATOR_HPP

#include <DFNCommonInterface.hpp>
#include <map>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFPC
{
	/**
	 * This class handles the common configuration operation for each DFPC.
	 *
	 * The DfpcConfigurator adds the following capability:
	 * (i) automatic splitting of DFPC configuration file in multiple configuration files, one for each DFN so that each DFN can access its proper configuration file;
	 * (ii) instantiation of each DFN according to the information written in the DFNsChain configuration file
	 *
	 * With the above functionality, a DFPC will need to
	 * acquire access to the instantiated DFNs;
	 * acquire extra DFPC configuration parameters beyond those used by the DFNs.
	 */
	class DfpcConfigurator
	{
	public:

		DfpcConfigurator();
		~DfpcConfigurator();

		/*
		* @brief this method instantiates the DFNs and configures them according to the information available in the configuration file. It also creates an extra file for the DFPC extra parameters.
		*
		* @param configurationFilePath, this is the path to the configuration file.
		*/
		void configure(std::string configurationFilePath);
		/*
	   	 * @brief this method  creates an extra file for the DFPC extra parameters.
	    	 *
	    	 * @param configurationFilePath, this is the path to the configuration file.
	    	*/
	   	std::string configurePath(std::string configurationFilePath);

		/*
		* @brief allows you to retrieve the extra configuration file for the extra DFPC configuration parameters
		*
		* @output extraFilePath, this is the path to the extra configuration file.
		*/
		std::string GetExtraParametersConfigurationFilePath();

		/*
		* @brief this method allows you to get the DFN instance instantiated by the configure method.
		*
		* @param dfnName, the name of the DFN as mentioned in the original configuration file.
		* @param optional, if this parameter is false, an error will be generated if the dfn was not successfully instantiated, if the parameter is true the method will return NULL if the dfn was not
		*		previously generated.
		*/
		CDFF::DFN::DFNCommonInterface* GetDfn(std::string dfnName, bool optional = false);

	protected:
		std::string extraParametersConfigurationFilePath;

		std::map<std::string, CDFF::DFN::DFNCommonInterface*> dfnsSet;
		std::map<std::string, std::string> configurationFilesSet;

	private:
		void ConstructDFNs(YAML::Node configuration);
		void SplitConfigurationFile(YAML::Node configuration,  std::string folderPath);
		std::string ComputeConfigurationFolderPath(std::string configurationFilePath);
		void ConfigureDfns();
		void DestroyDfns();
	};
}
}

#endif // DFPC_CONFIGURATOR_HPP

/** @} */
