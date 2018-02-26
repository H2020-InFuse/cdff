/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsChainInterface.cpp
 * @date 23/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the DFNsChainInterface non virtual methods.
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
#include "DFNsChainInterface.hpp"
#include "Errors/Assert.hpp"
#include <fstream>
#include <DFNsBuilder.hpp>

namespace dfpc_ci {

using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DFNsChainInterface::DFNsChainInterface() : outputUpdated(true) 
	{

	}
DFNsChainInterface::~DFNsChainInterface() 
	{
	DestroyDFNs();
	}

void DFNsChainInterface::configure() 
	{
	try
		{
		YAML::Node configuration= YAML::LoadFile( configurationFilePath );
		if (dfnsSet.empty())
			{
			ConstructDFNs(configuration);
			}
		SplitConfigurationFile(configuration);
		ConfigureDfns();
		AssignDfnsAlias();
		} 
	catch(YAML::Exception& e) 
		{
    		ASSERT(false, e.what() );
		}
	} 

bool DFNsChainInterface::hasNewOutput()
	{
	return outputUpdated;
	}

void DFNsChainInterface::executionTimeInput(int64_t data)
	{
	executionTime = data;
	}

void DFNsChainInterface::loggingIsActivatedInput(LogLevel data)
	{
	logLevel = data;
	}

void DFNsChainInterface::setConfigurationFile(std::string configurationFilePath)
	{
	this->configurationFilePath = configurationFilePath;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void DFNsChainInterface::ConstructDFNs(YAML::Node configuration)
	{
	for(unsigned dfnIndex = 0; dfnIndex < configuration.size(); dfnIndex++)
		{
		YAML::Node dfnNode = configuration[dfnIndex];

		std::string dfnName = dfnNode["Name"].as<std::string>();
		std::string dfnType = dfnNode["Type"].as<std::string>();
		std::string dfnImplementation = dfnNode["Implementation"].as<std::string>();

		DFNCommonInterface* dfn = DFNsBuilder::CreateDFN(dfnType, dfnImplementation);
		dfnsSet[dfnName] = dfn;
		}
	}

void DFNsChainInterface::SplitConfigurationFile(YAML::Node configuration)
	{
	std::string folderPath = ComputeConfigurationFolderPath();

	for(unsigned dfnIndex = 0; dfnIndex < configuration.size(); dfnIndex++)
		{
		std::stringstream nodeFileStream;
		nodeFileStream << folderPath << "/DFN_" << dfnIndex << ".yaml";

		YAML::Node dfnNode = configuration[dfnIndex];
		std::string dfnName = dfnNode["Name"].as<std::string>();

		YAML::Node parametersNode = dfnNode["Parameters"];

		std::ofstream file(nodeFileStream.str().c_str());
		file << parametersNode;
		file.close();

		configurationFilesSet[dfnName] = nodeFileStream.str();
		}
	}

std::string DFNsChainInterface::ComputeConfigurationFolderPath()
	{
 	static const char slash = '/';

   	unsigned lastSlashIndex = configurationFilePath.rfind(slash, configurationFilePath.length());
   	if (lastSlashIndex != std::string::npos) 
		{
      		return configurationFilePath.substr(0, lastSlashIndex);
   		}
	return "";
	}
		
void DFNsChainInterface::ConfigureDfns()
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

void DFNsChainInterface::DestroyDFNs()
	{
	for(std::map<std::string, DFNCommonInterface*>::iterator dfnsIterator = dfnsSet.begin(); dfnsIterator != dfnsSet.end(); dfnsIterator++)
		{
		DFNCommonInterface* dfn = dfnsIterator->second;
		delete(dfn);
		}
	dfnsSet.clear();
	configurationFilesSet.clear();
	}

void DFNsChainInterface::AssignDfnsAlias()	
	{

	}

}


/** @} */
