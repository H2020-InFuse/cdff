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
	}

void StructureFromMotion::process() 
	{
	DEBUG_WRITE_TO_LOG("Structure from motion start", "");
	currentImage = inImage;
	map->AddFrame(currentImage);
	DEBUG_WRITE_TO_LOG("Added Frame", "");
	FilterCurrentImage();
	DEBUG_WRITE_TO_LOG("Filtered Frame", "");
	ExtractCurrentFeatures();
	DEBUG_WRITE_TO_LOG("Extracted Features", GetNumberOfPoints(*currentKeypointsVector) );
	DescribeCurrentFeatures();
	DEBUG_WRITE_TO_LOG("Described Features", GetNumberOfPoints(*currentFeaturesVector) );

	bool success = false;
	for(pastImage = map->GetNextReferenceFrame(); !success && pastImage != NULL; pastImage = map->GetNextReferenceFrame())
		{
		DEBUG_WRITE_TO_LOG("Selected Past Frame", success);
		FilterPastImage();
		DEBUG_WRITE_TO_LOG("Filtered Frame", "");
		ExtractPastFeatures();
		DEBUG_WRITE_TO_LOG("Extracted Features", GetNumberOfPoints(*pastKeypointsVector) );
		DescribePastFeatures();
		DEBUG_WRITE_TO_LOG("Described Features", GetNumberOfPoints(*pastFeaturesVector) );

		MatchCurrentAndPastFeatures();
		DEBUG_WRITE_TO_LOG("Correspondences", GetNumberOfCorrespondences(*correspondenceMap) );
		success = ComputeFundamentalMatrix();
		DEBUG_WRITE_TO_LOG("Fundamental Matrix", success);
		if (success)
			{
			success = ComputePastToCurrentTransform();
			DEBUG_WRITE_TO_LOG("Essential Matrix", success);
			}
		if (success)
			{	
			ComputePointCloud();	
			DEBUG_WRITE_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
			}
		}

	if (success)
		{
		map->AddFramePose(pastToCurrentCameraTransform);
		map->AddPointCloudInLastReference(pointCloud);
		sceneCloud = map->GetPartialScene(searchRadius);
		DEBUG_WRITE_TO_LOG("Scene Cloud", GetNumberOfPoints(*sceneCloud));
		outPointCloud = sceneCloud;
		}
	else
		{
		outPointCloud = NULL;
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

	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>(dfnsSet["featureExtractor3d"]);
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>(dfnsSet["featuresMatcher3d"]);

	if (dfnsSet.find("featuresDescriptor3d") != dfnsSet.end() )
		{
		optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>(dfnsSet["featuresDescriptor"]);
		optionalDFNsSet++;
		}
	else
		{
		optionalFeaturesDescriptor3d = NULL;
		}

	ASSERT(dfnNumber == mandatoryDFNsNumber + optionalDFNsSet, "DFPC Structure from motion error: wrong number of DFNs in configuration file");
	}

void StructureFromMotion::FilterCurrentImage()
	{
	filter->imageInput(currentImage);
	filter->process();
	DELETE_PREVIOUS(filteredCurrentImage);
	filteredCurrentImage = filter->filteredImageOutput();
	}

void StructureFromMotion::FilterPastImage()
	{
	filter->imageInput(pastImage);
	filter->process();
	DELETE_PREVIOUS(filteredPastImage);
	filteredPastImage = filter->filteredImageOutput();
	}

void StructureFromMotion::ExtractCurrentFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentKeypointsVector);
	currentKeypointsVector = featuresExtractor->featuresSetOutput();
	}

void StructureFromMotion::ExtractPastFeatures()
	{
	featuresExtractor->imageInput(filteredPastImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(pastKeypointsVector);
	pastKeypointsVector = featuresExtractor->featuresSetOutput();
	}

void StructureFromMotion::DescribeCurrentFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(currentKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(currentFeaturesVector);
		currentFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
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
	}

bool StructureFromMotion::ComputeFundamentalMatrix()
	{
	fundamentalMatrixComputer->correspondenceMapInput(correspondenceMap);
	fundamentalMatrixComputer->process();
	DELETE_PREVIOUS(fundamentalMatrix);	
	fundamentalMatrix = fundamentalMatrixComputer->fundamentalMatrixOutput();	
	return fundamentalMatrixComputer->successOutput();
	}

bool StructureFromMotion::ComputePastToCurrentTransform()
	{
	cameraTransformEstimator->fundamentalMatrixInput(fundamentalMatrix);
	cameraTransformEstimator->correspondenceMapInput(correspondenceMap);
	cameraTransformEstimator->process();
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	pastToCurrentCameraTransform = cameraTransformEstimator->transformOutput();
	return cameraTransformEstimator->successOutput();
	}

void StructureFromMotion::ComputePointCloud()
	{
	reconstructor3D->poseInput(pastToCurrentCameraTransform);
	reconstructor3D->correspondenceMapInput(correspondenceMap);
	reconstructor3D->process();
	DELETE_PREVIOUS(pointCloud);
	pointCloud = reconstructor3D->pointCloudOutput();
	}

void StructureFromMotion::ExtractSceneFeatures()
	{
	featuresExtractor3d->pointCloudInput(pointCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(sceneKeypointsVector);
	sceneKeypointsVector = featuresExtractor3d->featuresSetOutput();
	}

void StructureFromMotion::ExtractModelFeatures()
	{
	featuresExtractor3d->pointCloudInput(modelCloud);
	featuresExtractor3d->process();
	DELETE_PREVIOUS(modelKeypointsVector);
	sceneKeypointsVector = featuresExtractor3d->featuresSetOutput();
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
		optionalFeaturesDescriptor3d->pointCloudInput(modelCloud);
		optionalFeaturesDescriptor3d->featuresSetInput(modelKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		DELETE_PREVIOUS(modelFeaturesVector);
		modelFeaturesVector = optionalFeaturesDescriptor3d->featuresSetWithDescriptorsOutput();
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
	return featuresMatcher3d->successOutput();
	}
}


/** @} */
