/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DfpcConfigurator.cpp
 * @date 21/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the DfpcConfigurator class.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "DfpcConfigurator.hpp"
#include "Errors/Assert.hpp"
#include <fstream>
#include <DFNsBuilder.hpp>

namespace CDFF
{
namespace DFPC
{

using namespace CDFF::DFN;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DfpcConfigurator::DfpcConfigurator()
	{

	}

DfpcConfigurator::~DfpcConfigurator() 
	{
	DestroyDfns();
	}

void DfpcConfigurator::configure(std::string configurationFilePath) 
	{
	try
		{
		std::string folderPath = ComputeConfigurationFolderPath(configurationFilePath);
		YAML::Node configuration= YAML::LoadFile( configurationFilePath );
		SplitConfigurationFile(configuration, folderPath);
		if (dfnsSet.empty())
			{
			ConstructDFNs(configuration);
			}
		ConfigureDfns();
		} 
	catch(YAML::Exception& e) 
		{
    		ASSERT(false, e.what() );
		}
	} 

std::string DfpcConfigurator::GetExtraParametersConfigurationFilePath()
	{
	return extraParametersConfigurationFilePath;
	}

DFNCommonInterface* DfpcConfigurator::GetDfn(std::string dfnName, bool optional)
	{
	std::map<std::string, DFNCommonInterface*>::iterator dfnElement = dfnsSet.find(dfnName);
	if ( dfnElement != dfnsSet.end() )
		{
		return dfnElement->second;
		}
	else
		{
		ASSERT(optional, "Mandatory DFN not properly configured");
		return NULL;
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void DfpcConfigurator::ConstructDFNs(YAML::Node configuration)
	{
	for(unsigned dfnIndex = 0; dfnIndex < configuration.size(); dfnIndex++)
		{
		YAML::Node dfnNode = configuration[dfnIndex];
		std::string dfnName = dfnNode["Name"].as<std::string>();
		
		if (dfnName == "DFNsChain")
			{
			continue;
			}

		std::string dfnType = dfnNode["Type"].as<std::string>();
		std::string dfnImplementation = dfnNode["Implementation"].as<std::string>();

		DFNCommonInterface* dfn = DFNsBuilder::CreateDFN(dfnType, dfnImplementation);
		dfnsSet[dfnName] = dfn;
		}
	}

void DfpcConfigurator::SplitConfigurationFile(YAML::Node configuration, std::string folderPath)
	{
	for(unsigned dfnIndex = 0; dfnIndex < configuration.size(); dfnIndex++)
		{
		YAML::Node dfnNode = configuration[dfnIndex];
		std::string dfnName = dfnNode["Name"].as<std::string>();

		std::stringstream nodeFileStream;
		nodeFileStream << folderPath << "/" << (dfnName == "DFNsChain" ? "" : "DFN_") << dfnName << ".yaml";
		std::string string = nodeFileStream.str();

		YAML::Node parametersNode = dfnNode["Parameters"];

		std::ofstream file(nodeFileStream.str().c_str());
		file << parametersNode;
		file.close();

		if (dfnName == "DFNsChain")
			{
			extraParametersConfigurationFilePath = nodeFileStream.str();
			}
		else
			{
			configurationFilesSet[dfnName] = nodeFileStream.str();
			}
		}
	}

std::string DfpcConfigurator::ComputeConfigurationFolderPath(std::string configurationFilePath)
	{
 	static const char slash = '/';

   	unsigned lastSlashIndex = configurationFilePath.rfind(slash, configurationFilePath.length());
   	if (lastSlashIndex != std::string::npos) 
		{
      		return configurationFilePath.substr(0, lastSlashIndex);
   		}
	return "";
	}
		
void DfpcConfigurator::ConfigureDfns()
	{
	for(std::map<std::string, DFNCommonInterface*>::iterator dfnsIterator = dfnsSet.begin(); dfnsIterator != dfnsSet.end(); dfnsIterator++)
		{
		std::string dfnName = dfnsIterator->first;
		DFNCommonInterface* dfn = dfnsIterator->second;
		ASSERT( configurationFilesSet.find(dfnName) != configurationFilesSet.end(), "DFNsChainInterface Error: no configuration file set for a dfn");
		dfn->setConfigurationFile( configurationFilesSet[dfnName] );
		dfn->configure();
		}
	}

void DfpcConfigurator::DestroyDfns()
	{
	for(std::map<std::string, DFNCommonInterface*>::iterator dfnsIterator = dfnsSet.begin(); dfnsIterator != dfnsSet.end(); dfnsIterator++)
		{
		DFNCommonInterface* dfn = dfnsIterator->second;
		delete(dfn);
		}
	dfnsSet.clear();
	configurationFilesSet.clear();
	}

}
}

/** @} */
