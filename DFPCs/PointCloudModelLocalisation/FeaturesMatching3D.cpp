/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.cpp
 * @date 23/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesMatching3D class.
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
#include "FeaturesMatching3D.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

#include <Executors/FeaturesExtraction3D/FeaturesExtraction3DExecutor.hpp>
#include <Executors/FeaturesDescription3D/FeaturesDescription3DExecutor.hpp>
#include <Executors/FeaturesMatching3D/FeaturesMatching3DExecutor.hpp>

namespace CDFF
{
namespace DFPC
{
namespace PointCloudModelLocalisation
{

using namespace CDFF::DFN;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesMatching3D::FeaturesMatching3D()
	{
	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

	modelFeatureVector = NewVisualPointFeatureVector3D();

	configurationFilePath = "";
	modelFeaturesAvailable = false;
	}

FeaturesMatching3D::~FeaturesMatching3D()
	{
	DeleteIfNotNull(featuresExtractor3d);
	DeleteIfNotNull(optionalFeaturesDescriptor3d);
	DeleteIfNotNull(featuresMatcher3d);

	DeleteIfNotNull(modelFeatureVector);
	}

void FeaturesMatching3D::run() 
	{
	DEBUG_PRINT_TO_LOG("FeaturesMatching3D start", "");

	if (!modelFeaturesAvailable)
		{	
		VisualPointFeatureVector3DConstPtr modelKeypointVector = NULL;
		Executors::Execute(featuresExtractor3d, inModel, modelKeypointVector);
		Executors::Execute(optionalFeaturesDescriptor3d, &inModel, modelKeypointVector, modelFeatureVector);
		modelFeaturesAvailable = true;
		}


	VisualPointFeatureVector3DConstPtr sceneKeypointVector = NULL;
	VisualPointFeatureVector3DConstPtr sceneFeatureVector = NULL;
	Executors::Execute(featuresExtractor3d, inScene, sceneKeypointVector);
	Executors::Execute(optionalFeaturesDescriptor3d, &inScene, sceneKeypointVector, sceneFeatureVector);

	Executors::Execute(featuresMatcher3d, modelFeatureVector, sceneFeatureVector, &outPose, outSuccess);
	}

void FeaturesMatching3D::setup()
	{
	configurator.configure(configurationFilePath);
	InstantiateDFNs();
	}

void FeaturesMatching3D::modelInput(const asn1SccPointcloud& data)
	{
	modelFeaturesAvailable = false;
	PointCloudModelLocalisationInterface::modelInput(data);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void FeaturesMatching3D::InstantiateDFNs()
	{
	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") );
	optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) );
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") );
	}

}
}
}


/** @} */
