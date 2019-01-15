/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegistrationAndMatching.cpp
 * @date 31/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the RegistrationAndMatching class.
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
#include "RegistrationAndMatching.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>
#include <fstream>

namespace CDFF
{
namespace DFPC
{
namespace ReconstructionAndIdentification
{

using namespace CDFF::DFPC::Reconstruction3D;
using namespace CDFF::DFPC::PointCloudModelLocalisation;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
RegistrationAndMatching::RegistrationAndMatching()
	{
	registrationFromStereo = new RegistrationFromStereo();
	featuresMatching3d = new FeaturesMatching3D();
	modelFeaturesAvailable = false;
	}

RegistrationAndMatching::~RegistrationAndMatching()
	{
	delete(registrationFromStereo);
	delete(featuresMatching3d);
	}

void RegistrationAndMatching::run() 
	{
	registrationFromStereo->leftImageInput(inLeftImage);
	registrationFromStereo->rightImageInput(inRightImage);
	registrationFromStereo->run();
	outSuccess = registrationFromStereo->successOutput();

	if (outSuccess)
		{
		featuresMatching3d->sceneInput( registrationFromStereo->pointCloudOutput() );
		if (!modelFeaturesAvailable)
			{
			featuresMatching3d->modelInput( inModel );
			modelFeaturesAvailable = true;
			}
		featuresMatching3d->computeModelFeaturesInput( inComputeModelFeatures );
		featuresMatching3d->run();

		Copy( featuresMatching3d->poseOutput(), outPose);
		outSuccess = featuresMatching3d->successOutput();
		}
	}

void RegistrationAndMatching::setup()
	{
	std::ifstream configurationFile(configurationFilePath.c_str());
	
	std::string registrationFromStereoConfigurationFile, featuresMatching3dConfigurationFile;
	configurationFile >> registrationFromStereoConfigurationFile;
	configurationFile >> featuresMatching3dConfigurationFile;
	
	registrationFromStereo->setConfigurationFile( registrationFromStereoConfigurationFile );
	registrationFromStereo->setup();

	featuresMatching3d->setConfigurationFile( featuresMatching3dConfigurationFile );
	featuresMatching3d->setup();

	configurationFile.close();
	}

void RegistrationAndMatching::modelInput(const asn1SccPointcloud& data)
	{
	modelFeaturesAvailable = false;
	ReconstructionAndIdentificationInterface::modelInput(data);
	}


}
}
}


/** @} */
