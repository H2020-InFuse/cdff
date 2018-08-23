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

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		} \
	} \

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
		featuresExtractor3d->Execute(inModel, modelKeypointVector);
		optionalFeaturesDescriptor3d->Execute(&inModel, modelKeypointVector, modelFeatureVector);
		modelFeaturesAvailable = true;
		}


	VisualPointFeatureVector3DConstPtr sceneKeypointVector = NULL;
	VisualPointFeatureVector3DConstPtr sceneFeatureVector = NULL;
	featuresExtractor3d->Execute(inScene, sceneKeypointVector);
	optionalFeaturesDescriptor3d->Execute(&inScene, sceneKeypointVector, sceneFeatureVector);

	featuresMatcher3d->Execute(modelFeatureVector, sceneFeatureVector, &outPose, outSuccess);
	}

void FeaturesMatching3D::setup()
	{
	configurator.configure(configurationFilePath);
	InstantiateDFNExecutors();
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

void FeaturesMatching3D::InstantiateDFNExecutors()
	{
	featuresExtractor3d = new FeaturesExtraction3DExecutor( static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") ) );
	optionalFeaturesDescriptor3d = new FeaturesDescription3DExecutor( static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) ) );
	featuresMatcher3d = new FeaturesMatching3DExecutor( static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") ) );
	}

}
}
}


/** @} */
