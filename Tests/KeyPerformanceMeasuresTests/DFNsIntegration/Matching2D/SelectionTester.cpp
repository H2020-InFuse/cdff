/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.cpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the SelectionTester class.
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
#include "SelectionTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ctime>

using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;
using namespace Common;
using namespace CorrespondenceMap2DWrapper;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
SelectionTester::SelectionTester() 
	{
	featuresDescriptorConfigurationFilePath = "";
	featuresMatcherConfigurationFilePath = "";

	inputSourceFrame = NULL;
	inputSinkFrame = NULL;
	inputSourceKeypointsVector = NULL;
	inputSinkKeypointsVector = NULL;
	sourceFeaturesVector = NULL;
	sinkFeaturesVector = NULL;
	outputCorrespondenceMap = NULL;
	referenceCorrespondenceMap = NULL;

	matcher = NULL;
	descriptor = NULL;

	dfnsWereLoaded = false;
	inputImagesWereLoaded = false;
	inputKeypointsWereLoaded = false;
	precisionReferenceWasLoaded = false;

	SetUpMocksAndStubs();
	}

SelectionTester::~SelectionTester()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubVector2dCache);
	delete(mockVector2dConverter);

	delete(stubInverseVector2dCache);
	delete(mockInverseVector2dConverter);
	
	DELETE_IF_NOT_NULL(inputSourceFrame);
	DELETE_IF_NOT_NULL(inputSinkFrame);
	DELETE_IF_NOT_NULL(inputSourceKeypointsVector);
	DELETE_IF_NOT_NULL(inputSinkKeypointsVector);
	DELETE_IF_NOT_NULL(sourceFeaturesVector);
	DELETE_IF_NOT_NULL(sinkFeaturesVector);
	DELETE_IF_NOT_NULL(outputCorrespondenceMap);
	DELETE_IF_NOT_NULL(referenceCorrespondenceMap);
	}

void SelectionTester::SetDfns(dfn_ci::FeaturesDescription2DInterface* descriptor, dfn_ci::FeaturesMatching2DInterface* matcher)
	{
	this->descriptor = descriptor;
	this->matcher = matcher;

	if (featuresDescriptorConfigurationFilePath != "")
		{
		ConfigureDfns();
		}
	}

void SelectionTester::SetConfigurationFilePaths(std::string featuresDescriptorConfigurationFilePath, std::string featuresMatcherConfigurationFilePath)
	{
	this->featuresDescriptorConfigurationFilePath = featuresDescriptorConfigurationFilePath;
	this->featuresMatcherConfigurationFilePath = featuresMatcherConfigurationFilePath;

	if (descriptor != NULL && matcher != NULL)
		{
		ConfigureDfns();
		}
	}

void SelectionTester::SetInputFilesPaths(std::string sourceImageFilePath, std::string sinkImageFilePath, std::string correspondencesImageFilePath)
	{
	this->sourceImageFilePath = sourceImageFilePath;
	this->sinkImageFilePath = sinkImageFilePath;
	this->correspondencesImageFilePath = correspondencesImageFilePath;

	LoadInputImage(sourceImageFilePath, inputSourceFrame);
	LoadInputImage(sinkImageFilePath, inputSinkFrame);
	inputImagesWereLoaded = true;

	LoadReferenceCorrespondenceMap();
	inputKeypointsWereLoaded = true;
	precisionReferenceWasLoaded = true;
	}

#define PROCESS_AND_MEASURE_TIME(dfn) \
	{ \
	beginTime = clock(); \
	dfn->process(); \
	endTime = clock(); \
	processingTime += float(endTime - beginTime) / CLOCKS_PER_SEC; \
	}

void SelectionTester::ExecuteDfns()
	{
	clock_t beginTime, endTime;
	float processingTime = 0;

	descriptor->imageInput(inputSourceFrame);
	descriptor->featuresSetInput(inputSourceKeypointsVector);
	PROCESS_AND_MEASURE_TIME(descriptor);
	DELETE_IF_NOT_NULL(sourceFeaturesVector);
	sourceFeaturesVector = descriptor->featuresSetWithDescriptorsOutput();

	descriptor->imageInput(inputSinkFrame);
	descriptor->featuresSetInput(inputSinkKeypointsVector);
	PROCESS_AND_MEASURE_TIME(descriptor);
	DELETE_IF_NOT_NULL(sinkFeaturesVector);
	sinkFeaturesVector = descriptor->featuresSetWithDescriptorsOutput();

	matcher->sourceFeaturesVectorInput(sourceFeaturesVector);
	matcher->sinkFeaturesVectorInput(sinkFeaturesVector);
	PROCESS_AND_MEASURE_TIME(matcher);
	DELETE_IF_NOT_NULL(outputCorrespondenceMap);
	outputCorrespondenceMap = matcher->correspondenceMapOutput();	

	PRINT_TO_LOG("Processing took (seconds): ", processingTime);
	}

bool SelectionTester::AreCorrespondencesValid(float percentageThreshold)
	{
	ASSERT(inputImagesWereLoaded && inputKeypointsWereLoaded && precisionReferenceWasLoaded, "Error: some inputs were not correctly loaded");

	PRINT_TO_LOG("Source Image Keypoints are", GetNumberOfPoints(*inputSourceKeypointsVector));
	PRINT_TO_LOG("Sink Image Keypoints are", GetNumberOfPoints(*inputSinkKeypointsVector));

	bool correspondencesAreValid = ValidateCorrespondences(percentageThreshold);
	if (!correspondencesAreValid)
		{
		PRINT_TO_LOG("Correspondences are not valid according to percentage threshold", percentageThreshold);
		}

	return correspondencesAreValid;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void SelectionTester::SetUpMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>;
	mockFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>;
	mockInverseFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubVector2dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockVector2dConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubVector2dCache, mockVector2dConverter);

	stubInverseVector2dCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockInverseVector2dConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubInverseVector2dCache, mockInverseVector2dConverter);
	}

void SelectionTester::LoadInputImage(const std::string& imageFilePath, FrameWrapper::FrameConstPtr& frame)
	{
	cv::Mat cvImage = cv::imread(imageFilePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(frame);
	frame = frameConverter.Convert(cvImage);
	}

void SelectionTester::LoadReferenceCorrespondenceMap()
	{
	cv::FileStorage opencvFile(correspondencesImageFilePath, cv::FileStorage::READ);

	cv::Mat cvCorrespondenceMap;
	opencvFile["CorrespondenceMap"] >> cvCorrespondenceMap;
	opencvFile.release();

	ASSERT(cvCorrespondenceMap.rows > 0, "Error: correspondence map contains no keypoints");
	ASSERT(cvCorrespondenceMap.cols == 4, "Error: correspondence map has invalid format. It should contain a row for each correspondence, and four integers representing the image coordinates");
	ASSERT(cvCorrespondenceMap.type() == CV_16UC1, "Error: correspondence map invalid format. It should contain a row for each correspondence, and four integers representing the image coordinates");

	VisualPointFeatureVector2DPtr newInputSourceKeypointsVector = NewVisualPointFeatureVector2D();
	VisualPointFeatureVector2DPtr newInputSinkKeypointsVector = NewVisualPointFeatureVector2D();
	CorrespondenceMap2DPtr newReferenceCorrespondenceMap = NewCorrespondenceMap2D();

	for(unsigned row = 0; row < cvCorrespondenceMap.rows; row++)
		{
		BaseTypesWrapper::Point2D source, sink;
		source.x = cvCorrespondenceMap.at<uint16_t>(row, 0);
		source.y = cvCorrespondenceMap.at<uint16_t>(row, 1);
		sink.x = cvCorrespondenceMap.at<uint16_t>(row, 2);
		sink.y = cvCorrespondenceMap.at<uint16_t>(row, 3);

		AddPoint(*newInputSourceKeypointsVector, source.x, source.y);
		AddPoint(*newInputSinkKeypointsVector, sink.x, sink.y);
		AddCorrespondence(*newReferenceCorrespondenceMap, source, sink, 1);
		}

	inputSourceKeypointsVector = newInputSourceKeypointsVector;
	inputSinkKeypointsVector = newInputSinkKeypointsVector;
	referenceCorrespondenceMap = newReferenceCorrespondenceMap;
	}

void SelectionTester::ConfigureDfns()
	{
	descriptor->setConfigurationFile(featuresDescriptorConfigurationFilePath);
	descriptor->configure();

	matcher->setConfigurationFile(featuresMatcherConfigurationFilePath);
	matcher->configure();

	dfnsWereLoaded = true;
	}

bool SelectionTester::ValidateCorrespondences(float percentageThreshold)
	{
	int outputCorrespondecesNumber = GetNumberOfCorrespondences(*outputCorrespondenceMap);
	int referenceCorrespondencesNumber = GetNumberOfCorrespondences(*referenceCorrespondenceMap);

	int correctMatchesNumber = 0;
	for(int referenceIndex = 0; referenceIndex < referenceCorrespondencesNumber; referenceIndex++)
		{
		for (int outputIndex = 0; outputIndex < outputCorrespondecesNumber; outputIndex++)
			{
			if (CorrespondencesAreTheSame(referenceIndex, outputIndex))
				{
				correctMatchesNumber++;
				}
			}
		}
	float correctMatchesPercentage = ((float)correctMatchesNumber) / ((float)referenceCorrespondencesNumber);
	PRINT_TO_LOG("Percentage of correct matches", correctMatchesPercentage);

	return (correctMatchesPercentage >= percentageThreshold);
	}

bool SelectionTester::CorrespondencesAreTheSame(int referenceIndex, int outputIndex)
	{
	BaseTypesWrapper::Point2D referenceSource = GetSource(*referenceCorrespondenceMap, referenceIndex);
	BaseTypesWrapper::Point2D referenceSink = GetSink(*referenceCorrespondenceMap, referenceIndex);		
	BaseTypesWrapper::Point2D outputSource = GetSource(*outputCorrespondenceMap, outputIndex);
	BaseTypesWrapper::Point2D outputSink = GetSink(*outputCorrespondenceMap, outputIndex);	

	return (referenceSource.x == outputSource.x && referenceSource.y == outputSource.y && referenceSink.x == outputSink.x && referenceSink.y == outputSink.y);
	}

/** @} */
