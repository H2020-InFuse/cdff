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

namespace dfpc_ci {

using namespace dfn_ci;
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
	sceneCloud = NULL;
	lastModelCloud = NULL;
	sceneKeypointsVector = NewVisualPointFeatureVector3D();
	modelKeypointsVector = NewVisualPointFeatureVector3D();
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;

	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

	configurationFilePath = "";
	}

FeaturesMatching3D::~FeaturesMatching3D()
	{
	delete(sceneKeypointsVector);
	delete(modelKeypointsVector);
	if (optionalFeaturesDescriptor3d != NULL)
		{
		DELETE_PREVIOUS(sceneFeaturesVector);
		DELETE_PREVIOUS(modelFeaturesVector);
		}
	}

void FeaturesMatching3D::run() 
	{
	DEBUG_PRINT_TO_LOG("FeaturesMatching3D start", "");
	sceneCloud = &inScene;

	ExtractSceneFeatures();
	DescribeSceneFeatures();

	if (lastModelCloud != &inModel || inComputeModelFeatures)
		{
		lastModelCloud = &inModel;
		ExtractModelFeatures();
		DescribeModelFeatures();		
		}
	outSuccess = EstimateModelPose();
	}

void FeaturesMatching3D::setup()
	{
	configurator.configure(configurationFilePath);
	AssignDfnsAlias();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void FeaturesMatching3D::AssignDfnsAlias()
	{
	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") );
	optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) );
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") );

	ASSERT(featuresExtractor3d != NULL, "DFPC Structure from motion error: featuresExtractor3d DFN configured incorrectly");
	ASSERT(featuresMatcher3d != NULL, "DFPC Structure from motion error: featuresMatcher3d DFN configured incorrectly");

	if (optionalFeaturesDescriptor3d != NULL)
		{
		sceneFeaturesVector = NewVisualPointFeatureVector3D();
		modelFeaturesVector = NewVisualPointFeatureVector3D();
		}
	}

void FeaturesMatching3D::ExtractSceneFeatures()
	{
	featuresExtractor3d->pointcloudInput(*sceneCloud);
	featuresExtractor3d->process();
	Copy( featuresExtractor3d->featuresOutput(), *sceneKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Scene Features", GetNumberOfPoints(*sceneKeypointsVector));
	DEBUG_SHOW_3D_VISUAL_FEATURES(sceneCloud, sceneKeypointsVector);
	}

void FeaturesMatching3D::ExtractModelFeatures()
	{
	featuresExtractor3d->pointcloudInput(*lastModelCloud);
	featuresExtractor3d->process();
	Copy( featuresExtractor3d->featuresOutput(), *modelKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Model Features", GetNumberOfPoints(*modelKeypointsVector));
	DEBUG_SHOW_3D_VISUAL_FEATURES(lastModelCloud, modelKeypointsVector);
	}

void FeaturesMatching3D::DescribeSceneFeatures()
	{
	if (optionalFeaturesDescriptor3d != NULL)
		{
		optionalFeaturesDescriptor3d->pointcloudInput(*sceneCloud);
		optionalFeaturesDescriptor3d->featuresInput(*sceneKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		Copy( optionalFeaturesDescriptor3d->featuresOutput(), *sceneFeaturesVector);
		DEBUG_PRINT_TO_LOG("Described Scene Features", GetNumberOfPoints(*sceneFeaturesVector));
		}
	else
		{
		sceneFeaturesVector = sceneKeypointsVector;
		}
	}

void FeaturesMatching3D::DescribeModelFeatures()
	{
	if (optionalFeaturesDescriptor3d != NULL)
		{
		optionalFeaturesDescriptor3d->pointcloudInput(*lastModelCloud);
		optionalFeaturesDescriptor3d->featuresInput(*modelKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		Copy( optionalFeaturesDescriptor3d->featuresOutput(), *modelFeaturesVector);
		DEBUG_PRINT_TO_LOG("Described Model Features", GetNumberOfPoints(*modelFeaturesVector));
		}
	else
		{
		modelFeaturesVector = modelKeypointsVector;
		}
	}

bool FeaturesMatching3D::EstimateModelPose()
	{
	featuresMatcher3d->sourceFeaturesInput(*modelFeaturesVector);
	featuresMatcher3d->sinkFeaturesInput(*sceneFeaturesVector);
	featuresMatcher3d->process();
	Copy( featuresMatcher3d->transformOutput(), outPose);
	bool matching3dSuccess = featuresMatcher3d->successOutput();
	DEBUG_PRINT_TO_LOG("Matching 3d", matching3dSuccess);
	if(matching3dSuccess)
		{
		DEBUG_SHOW_POSE(&outPose);
		DEBUG_PLACE_POINT_CLOUD(sceneCloud, lastModelCloud, &outPose);
		}
	return matching3dSuccess;
	}
}


/** @} */
