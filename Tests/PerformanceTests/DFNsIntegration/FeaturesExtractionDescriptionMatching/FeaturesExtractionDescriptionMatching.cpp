/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtractionDescriptionMatching.cpp
 * @date 27/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the class FeaturesExtractionDescriptionMatchingTestInterface
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
#include "FeaturesExtractionDescriptionMatching.hpp"

#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>

using namespace CDFF::DFN;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace CDFF::DFN::Executors;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesExtractionDescriptionMatching::FeaturesExtractionDescriptionMatching(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, 
	const std::string& performanceMeasuresFileName) :
	PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	leftFrame = NULL;
	rightFrame = NULL;
	leftFeaturesVectorHolder = NewVisualPointFeatureVector2D();
	correspondenceMap = NewCorrespondenceMap2D();
	}

FeaturesExtractionDescriptionMatching::~FeaturesExtractionDescriptionMatching()
	{
	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	delete(leftFeaturesVectorHolder);
	delete(correspondenceMap);
	}

void FeaturesExtractionDescriptionMatching::SetInputFiles(const std::string& leftImageFilePath, const std::string& rightImageFilePath)
	{
	this->leftImageFilePath = leftImageFilePath;
	this->rightImageFilePath = rightImageFilePath;
	}

void FeaturesExtractionDescriptionMatching::SetDfns(FeaturesExtraction2DInterface* extractor, FeaturesDescription2DInterface* descriptor, FeaturesMatching2DInterface* matcher)
	{
	this->extractor = extractor;
	this->descriptor = descriptor;
	this->matcher = matcher;

	AddDfn(extractor);
	if (descriptor != NULL)
		{
		AddDfn(descriptor);
		}
	AddDfn(matcher);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
bool FeaturesExtractionDescriptionMatching::SetNextInputs()
	{
	static unsigned time = 0;

	if (time == 0)
		{
		cvLeftImage = cv::imread(leftImageFilePath, cv::IMREAD_COLOR);
		cvRightImage = cv::imread(rightImageFilePath, cv::IMREAD_COLOR);
		ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");

		MatToFrameConverter converter;
		leftFrame = converter.Convert(cvLeftImage);
		rightFrame = converter.Convert(cvRightImage);

		time++;
		return true;
		}
	
	return false;
	}

void FeaturesExtractionDescriptionMatching::ExecuteDfns()
	{
	VisualPointFeatureVector2DConstPtr leftKeypointsVector = NULL;
	VisualPointFeatureVector2DConstPtr rightKeypointsVector = NULL;
	VisualPointFeatureVector2DConstPtr leftFeaturesVector = NULL;
	VisualPointFeatureVector2DConstPtr rightFeaturesVector = NULL;
	CorrespondenceMap2DConstPtr newCorrespondenceMap = NULL;

	Execute(extractor, leftFrame, leftKeypointsVector);
	Execute(descriptor, leftFrame, leftKeypointsVector, leftFeaturesVector);
	Copy(*leftFeaturesVector, *leftFeaturesVectorHolder);

	Execute(extractor, rightFrame, rightKeypointsVector);
	Execute(descriptor, rightFrame, rightKeypointsVector, rightFeaturesVector);
	Execute(matcher, leftFeaturesVectorHolder, rightFeaturesVector, newCorrespondenceMap);
	Copy(*newCorrespondenceMap, *correspondenceMap);
	}

FeaturesExtractionDescriptionMatching::MeasuresMap FeaturesExtractionDescriptionMatching::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	float outOfLineCost = 0;
	for(unsigned correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D source = GetSource(*correspondenceMap, correspondenceIndex); 
		BaseTypesWrapper::Point2D sink = GetSink(*correspondenceMap, correspondenceIndex);

 		outOfLineCost += std::abs( source.y - sink.y );
		}
	measuresMap["OutOfLineCost"] = (outOfLineCost / static_cast<float>( GetNumberOfCorrespondences(*correspondenceMap) ) );
	measuresMap["NumberOfCorrespondences"] = GetNumberOfCorrespondences(*correspondenceMap);

	return measuresMap;
	}

/** @} */
