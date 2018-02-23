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
StructureFromMotion::StructureFromMotion()
	{

	configurationFilePath = "";
	}

StructureFromMotion::~StructureFromMotion()
	{

	}

void StructureFromMotion::process() 
	{
	currentImage = inImage;
	imagesHistory.push_back(currentImage);
	FilterCurrentImage();
	ExtractCurrentFeatures();
	DescribeCurrentFeatures();

	bool success = false;
	for(int pastImageIndex = imagesHistory.size()-2; pastImageIndex >= 0 && !success; pastImageIndex--)
		{
		pastImage = imagesHistory.at(pastImageIndex);
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

	outSuccess = success;
	outPointCloud = pointCloud;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void StructureFromMotion::AssignDfnsAlias()
	{
	filter = static_cast<ImageFilteringInterface*>(dfnsSet["filter"]);
	featureExtractor = static_cast<FeaturesExtraction2DInterface*>(dfnsSet["featureExtractor"]);
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>(dfnsSet["featuresMatcher"]);
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>(dfnsSet["fundamentalMatrixComputer"]);
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>(dfnsSet["cameraTransformEstimator"]);
	reconstructor3D = static_cast<PointCloudReconstruction2DTo3DInterface*>(dfnsSet["reconstructor3D"]);

	if (dfnsSet.find("optionalFeatureDetector") != dfnsSet.end() )
		{
		optionalFeatureDescriptor = static_cast<FeaturesDescription2DInterface*>(dfnsSet["optionalFeatureDetector"]);
		}
	else
		{
		optionalFeatureDescriptor = NULL;
		}
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
	featureExtractor->imageInput(filteredCurrentImage);
	featureExtractor->process();
	DELETE_PREVIOUS(currentKeypointsVector);
	currentKeypointsVector = featureExtractor->featuresSetOutput();
	}

void StructureFromMotion::ExtractPastFeatures()
	{
	featureExtractor->imageInput(filteredPastImage);
	featureExtractor->process();
	DELETE_PREVIOUS(pastKeypointsVector);
	pastKeypointsVector = featureExtractor->featuresSetOutput();
	}

void StructureFromMotion::DescribeCurrentFeatures()
	{
	if (optionalFeatureDescriptor != NULL)
		{
		optionalFeatureDescriptor->featuresSetInput(currentKeypointsVector);
		optionalFeatureDescriptor->process();
		DELETE_PREVIOUS(currentFeaturesVector);
		currentFeaturesVector = optionalFeatureDescriptor->featuresSetWithDescriptorsOutput();
		}
	else
		{
		currentFeaturesVector = currentKeypointsVector;
		}
	}

void StructureFromMotion::DescribePastFeatures()
	{
	if (optionalFeatureDescriptor != NULL)
		{
		optionalFeatureDescriptor->featuresSetInput(pastKeypointsVector);
		optionalFeatureDescriptor->process();
		DELETE_PREVIOUS(pastFeaturesVector);
		pastFeaturesVector = optionalFeatureDescriptor->featuresSetWithDescriptorsOutput();
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
}


/** @} */
