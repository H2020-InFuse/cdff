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
	sceneKeypointsVector = NULL;
	modelKeypointsVector = NULL;
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;
	modelPoseInScene = NULL;

	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

	configurationFilePath = "";
	}

FeaturesMatching3D::~FeaturesMatching3D()
	{
	DELETE_PREVIOUS(sceneKeypointsVector);
	DELETE_PREVIOUS(modelKeypointsVector);
	if (optionalFeaturesDescriptor3d != NULL)
		{
		DELETE_PREVIOUS(sceneFeaturesVector);
		DELETE_PREVIOUS(modelFeaturesVector);
		}
	DELETE_PREVIOUS(modelPoseInScene);
	}

void FeaturesMatching3D::process() 
	{
	DEBUG_PRINT_TO_LOG("FeaturesMatching3D start", "");
	sceneCloud = inScene;

	ExtractSceneFeatures();
	DescribeSceneFeatures();

	if (lastModelCloud != inModel)
		{
		lastModelCloud = inModel;
		ExtractModelFeatures();
		DescribeModelFeatures();		
		}
	outSuccess = EstimateModelPose();

	if (!outSuccess)
		{
		outPose = NULL;
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void FeaturesMatching3D::AssignDfnsAlias()
	{
	unsigned dfnNumber = dfnsSet.size();
	unsigned mandatoryDFNsNumber = 2;
	unsigned optionalDFNsSet = 0;

	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>(dfnsSet["featuresExtractor3d"]);
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>(dfnsSet["featuresMatcher3d"]);

	if (dfnsSet.find("featuresDescriptor3d") != dfnsSet.end() )
		{
		optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>(dfnsSet["featuresDescriptor3d"]);
		ASSERT(optionalFeaturesDescriptor3d != NULL, "DFPC Structure from motion error: featuresDescriptor3d DFN configured incorrectly");
		optionalDFNsSet++;
		}
	else
		{
		optionalFeaturesDescriptor3d = NULL;
		}

	ASSERT(dfnNumber == mandatoryDFNsNumber + optionalDFNsSet, "DFPC Structure from motion error: wrong number of DFNs in configuration file");
	ASSERT(featuresExtractor3d != NULL, "DFPC Structure from motion error: featuresExtractor3d DFN configured incorrectly");
	ASSERT(featuresMatcher3d != NULL, "DFPC Structure from motion error: featuresMatcher3d DFN configured incorrectly");
	}

void FeaturesMatching3D::ExtractSceneFeatures()
	{
	featuresExtractor3d->pointCloudInput(sceneCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(sceneKeypointsVector);
	sceneKeypointsVector = featuresExtractor3d->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Scene Features", GetNumberOfPoints(*sceneKeypointsVector));
	DEBUG_SHOW_3D_VISUAL_FEATURES(sceneCloud, sceneKeypointsVector);
	}

void FeaturesMatching3D::ExtractModelFeatures()
	{
	featuresExtractor3d->pointCloudInput(lastModelCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(modelKeypointsVector);
	modelKeypointsVector = featuresExtractor3d->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Model Features", GetNumberOfPoints(*modelKeypointsVector));
	DEBUG_SHOW_3D_VISUAL_FEATURES(lastModelCloud, modelKeypointsVector);
	}

void FeaturesMatching3D::DescribeSceneFeatures()
	{
	if (optionalFeaturesDescriptor3d != NULL)
		{
		optionalFeaturesDescriptor3d->pointCloudInput(sceneCloud);
		optionalFeaturesDescriptor3d->featuresSetInput(sceneKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		DELETE_PREVIOUS(sceneFeaturesVector);
		sceneFeaturesVector = optionalFeaturesDescriptor3d->featuresSetWithDescriptorsOutput();
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
		optionalFeaturesDescriptor3d->pointCloudInput(lastModelCloud);
		optionalFeaturesDescriptor3d->featuresSetInput(modelKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		DELETE_PREVIOUS(modelFeaturesVector);
		modelFeaturesVector = optionalFeaturesDescriptor3d->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Model Features", GetNumberOfPoints(*modelFeaturesVector));
		}
	else
		{
		modelFeaturesVector = modelKeypointsVector;
		}
	}

bool FeaturesMatching3D::EstimateModelPose()
	{
	featuresMatcher3d->sourceFeaturesVectorInput(modelFeaturesVector);
	featuresMatcher3d->sinkFeaturesVectorInput(sceneFeaturesVector);
	DELETE_PREVIOUS(modelPoseInScene);
	featuresMatcher3d->process();
	modelPoseInScene = featuresMatcher3d->transformOutput();
	bool matching3dSuccess = featuresMatcher3d->successOutput();
	DEBUG_PRINT_TO_LOG("Matching 3d", matching3dSuccess);
	if(matching3dSuccess)
		{
		DEBUG_SHOW_POSE(modelPoseInScene);
		DEBUG_PLACE_POINT_CLOUD(sceneCloud, lastModelCloud, modelPoseInScene);
		}
	return matching3dSuccess;
	}
}


/** @} */
