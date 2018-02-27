/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StructureFromMotion.cpp
 * @date 23/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the StructureFromMotion class.
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
#include "StructureFromMotion.hpp"
#include "Errors/Assert.hpp"

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		} \
	} \

namespace dfpc_ci {

using namespace dfn_ci;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
StructureFromMotion::StructureFromMotion(Map* map) :
	map(map)
	{
	filteredCurrentImage = NULL;
	filteredPastImage = NULL;
	currentKeypointsVector = NULL;
	pastKeypointsVector = NULL;
	currentFeaturesVector = NULL;
	pastFeaturesVector = NULL;
	correspondenceMap = NULL;
	fundamentalMatrix = NULL;
	pastToCurrentCameraTransform = NULL;
	pointCloud = NULL;
	sceneCloud = NULL;
	lastModelCloud = NULL;
	sceneKeypointsVector = NULL;
	modelKeypointsVector = NULL;
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;
	modelPoseInScene = NULL;

	filter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	cameraTransformEstimator = NULL;
	reconstructor3D = NULL;
	optionalFeaturesDescriptor = NULL;
	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

	searchRadius = 100;

	configurationFilePath = "";
	}

StructureFromMotion::~StructureFromMotion()
	{
	DELETE_PREVIOUS(filteredCurrentImage);
	DELETE_PREVIOUS(filteredPastImage);
	DELETE_PREVIOUS(currentKeypointsVector);
	DELETE_PREVIOUS(pastKeypointsVector);
	if (optionalFeaturesDescriptor != NULL)
		{
		DELETE_PREVIOUS(currentFeaturesVector);
		DELETE_PREVIOUS(pastFeaturesVector);
		}
	DELETE_PREVIOUS(correspondenceMap);
	DELETE_PREVIOUS(fundamentalMatrix);
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	DELETE_PREVIOUS(pointCloud);
	DELETE_PREVIOUS(sceneCloud);
	DELETE_PREVIOUS(sceneKeypointsVector);
	DELETE_PREVIOUS(modelKeypointsVector);
	if (optionalFeaturesDescriptor3d != NULL)
		{
		DELETE_PREVIOUS(sceneFeaturesVector);
		DELETE_PREVIOUS(modelFeaturesVector);
		}
	DELETE_PREVIOUS(modelPoseInScene);
	}

void StructureFromMotion::process() 
	{
	DEBUG_PRINT_TO_LOG("Structure from motion start", "");
	currentImage = inImage;
	map->AddFrame(currentImage);
	FilterCurrentImage();
	ExtractCurrentFeatures();
	DescribeCurrentFeatures();

	bool success = false;
	for(pastImage = map->GetNextReferenceFrame(); !success && pastImage != NULL; pastImage = map->GetNextReferenceFrame())
		{
		DEBUG_PRINT_TO_LOG("Selected Past Frame", success);
		FilterPastImage();
		ExtractPastFeatures();
		DescribePastFeatures();

		MatchCurrentAndPastFeatures();
		success = ComputeFundamentalMatrix();
		if (success)
			{
			success = ComputePastToCurrentTransform();
			}
		if (success)
			{	
			ComputePointCloud();	
			}
		}

	if (success)
		{
		map->AddFramePose(pastToCurrentCameraTransform);
		map->AddPointCloudInLastReference(pointCloud);
		DELETE_PREVIOUS(sceneCloud);
		sceneCloud = map->GetPartialScene(searchRadius);
		DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*sceneCloud));
		outPointCloud = sceneCloud;
		ExtractSceneFeatures();
		DescribeSceneFeatures();
		if (lastModelCloud != inModel)
			{
			lastModelCloud = inModel;
			ExtractModelFeatures();
			DescribeModelFeatures();		
			}
		//success = EstimateModelPose();
		}
	else
		{
		outPointCloud = NULL;
		}

	if (success)
		{
		outPose = modelPoseInScene;
		}

	outSuccess = success;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void StructureFromMotion::AssignDfnsAlias()
	{
	unsigned dfnNumber = dfnsSet.size();
	unsigned mandatoryDFNsNumber = 8;
	unsigned optionalDFNsSet = 0;

	filter = static_cast<ImageFilteringInterface*>(dfnsSet["filter"]);
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>(dfnsSet["featureExtractor"]);
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>(dfnsSet["featuresMatcher"]);
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>(dfnsSet["fundamentalMatrixComputer"]);
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>(dfnsSet["cameraTransformEstimator"]);
	reconstructor3D = static_cast<PointCloudReconstruction2DTo3DInterface*>(dfnsSet["reconstructor3D"]);

	if (dfnsSet.find("featuresDescriptor") != dfnsSet.end() )
		{
		optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>(dfnsSet["featuresDescriptor"]);
		optionalDFNsSet++;
		}
	else
		{
		optionalFeaturesDescriptor = NULL;
		}

	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>(dfnsSet["featuresExtractor3d"]);
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>(dfnsSet["featuresMatcher3d"]);

	if (dfnsSet.find("featuresDescriptor3d") != dfnsSet.end() )
		{
		optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>(dfnsSet["featuresDescriptor3d"]);
		optionalDFNsSet++;
		}
	else
		{
		optionalFeaturesDescriptor3d = NULL;
		}

	ASSERT(dfnNumber == mandatoryDFNsNumber + optionalDFNsSet, "DFPC Structure from motion error: wrong number of DFNs in configuration file");
	ASSERT(filter != NULL, "DFPC Structure from motion error: filter DFN configured incorrectly");
	ASSERT(featuresExtractor != NULL, "DFPC Structure from motion error: featuresExtractor DFN configured incorrectly");
	ASSERT(featuresMatcher != NULL, "DFPC Structure from motion error: featuresMatcher DFN configured incorrectly");
	ASSERT(fundamentalMatrixComputer != NULL, "DFPC Structure from motion error: fundamentalMatrixComputer DFN configured incorrectly");
	ASSERT(cameraTransformEstimator != NULL, "DFPC Structure from motion error: cameraTransformEstimator DFN configured incorrectly");
	ASSERT(reconstructor3D != NULL, "DFPC Structure from motion error: reconstructor3D DFN configured incorrectly");
	ASSERT(featuresExtractor3d != NULL, "DFPC Structure from motion error: featuresExtractor3d DFN configured incorrectly");
	ASSERT(featuresMatcher3d != NULL, "DFPC Structure from motion error: featuresMatcher3d DFN configured incorrectly");
	}

void StructureFromMotion::FilterCurrentImage()
	{
	filter->imageInput(currentImage);
	filter->process();
	DELETE_PREVIOUS(filteredCurrentImage);
	filteredCurrentImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Frame", "");
	}

void StructureFromMotion::FilterPastImage()
	{
	filter->imageInput(pastImage);
	filter->process();
	DELETE_PREVIOUS(filteredPastImage);
	filteredPastImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Past Frame", "");
	}

void StructureFromMotion::ExtractCurrentFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentKeypointsVector);
	currentKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentKeypointsVector) );
	}

void StructureFromMotion::ExtractPastFeatures()
	{
	featuresExtractor->imageInput(filteredPastImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(pastKeypointsVector);
	pastKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastKeypointsVector) );
	}

void StructureFromMotion::DescribeCurrentFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(currentKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(currentFeaturesVector);
		currentFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Current Features", GetNumberOfPoints(*currentFeaturesVector) );
		}
	else
		{
		currentFeaturesVector = currentKeypointsVector;
		}
	}

void StructureFromMotion::DescribePastFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(pastKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(pastFeaturesVector);
		pastFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Past Features", GetNumberOfPoints(*pastFeaturesVector) );
		}
	else
		{
		pastFeaturesVector = pastKeypointsVector;
		}
	}

void StructureFromMotion::MatchCurrentAndPastFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(pastFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(currentFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(correspondenceMap);
	correspondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*correspondenceMap) );
	}

bool StructureFromMotion::ComputeFundamentalMatrix()
	{
	fundamentalMatrixComputer->correspondenceMapInput(correspondenceMap);
	fundamentalMatrixComputer->process();
	DELETE_PREVIOUS(fundamentalMatrix);	
	fundamentalMatrix = fundamentalMatrixComputer->fundamentalMatrixOutput();	
	bool fundamentalMatrixSuccess =  fundamentalMatrixComputer->successOutput();
	DEBUG_PRINT_TO_LOG("Fundamental Matrix", fundamentalMatrixSuccess);
	return fundamentalMatrixSuccess;
	}

bool StructureFromMotion::ComputePastToCurrentTransform()
	{
	cameraTransformEstimator->fundamentalMatrixInput(fundamentalMatrix);
	cameraTransformEstimator->correspondenceMapInput(correspondenceMap);
	cameraTransformEstimator->process();
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	pastToCurrentCameraTransform = cameraTransformEstimator->transformOutput();
	bool essentialMatrixSuccess = cameraTransformEstimator->successOutput();
	DEBUG_PRINT_TO_LOG("Essential Matrix", essentialMatrixSuccess);
	return essentialMatrixSuccess;
	}

void StructureFromMotion::ComputePointCloud()
	{
	reconstructor3D->poseInput(pastToCurrentCameraTransform);
	reconstructor3D->correspondenceMapInput(correspondenceMap);
	reconstructor3D->process();
	DELETE_PREVIOUS(pointCloud);
	pointCloud = reconstructor3D->pointCloudOutput();
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	}

void StructureFromMotion::ExtractSceneFeatures()
	{
	featuresExtractor3d->pointCloudInput(pointCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(sceneKeypointsVector);
	sceneKeypointsVector = featuresExtractor3d->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Scene Features", GetNumberOfPoints(*sceneKeypointsVector));
	}

void StructureFromMotion::ExtractModelFeatures()
	{
	featuresExtractor3d->pointCloudInput(lastModelCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(modelKeypointsVector);
	modelKeypointsVector = featuresExtractor3d->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Model Features", GetNumberOfPoints(*modelKeypointsVector));
	}

void StructureFromMotion::DescribeSceneFeatures()
	{
	if (optionalFeaturesDescriptor3d != NULL)
		{
		optionalFeaturesDescriptor3d->pointCloudInput(pointCloud);
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

void StructureFromMotion::DescribeModelFeatures()
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

bool StructureFromMotion::EstimateModelPose()
	{
	featuresMatcher3d->sourceFeaturesVectorInput(modelFeaturesVector);
	featuresMatcher3d->sinkFeaturesVectorInput(sceneFeaturesVector);
	DELETE_PREVIOUS(modelPoseInScene);
	modelPoseInScene = featuresMatcher3d->transformOutput();
	bool matching3dSuccess = featuresMatcher3d->successOutput();
	DEBUG_PRINT_TO_LOG("Matching 3d", matching3dSuccess);
	return matching3dSuccess;
	}
}


/** @} */
