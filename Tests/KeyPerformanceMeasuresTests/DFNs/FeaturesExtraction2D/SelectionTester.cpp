/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.cpp
 * @date 04/05/2018
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

using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;
using namespace Common;
using namespace PoseWrapper;

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
SelectionTester::SelectionTester(std::string configurationFilePath, dfn_ci::FeaturesExtraction2DInterface* dfn) 
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;

	inputFrame = NULL;
	outputFeaturesVector = NULL;

	inputImageWasLoaded = false;
	numberReferenceWasLoaded = false;
	precisionReferenceWasLoaded = false;

	SetUpMocksAndStubs();
	ConfigureDfn();
	}

SelectionTester::~SelectionTester()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubVector2dCache);
	delete(mockVector2dConverter);
	
	DELETE_IF_NOT_NULL(inputFrame);
	DELETE_IF_NOT_NULL(outputFeaturesVector);
	}

void SelectionTester::SetFilesPaths(std::string inputImageFilePath, std::string numberReferenceFilePath, std::string precisionReferenceFilePath)
	{
	this->inputImageFilePath = inputImageFilePath;
	this->numberReferenceFilePath = numberReferenceFilePath;
	this->precisionReferenceFilePath = precisionReferenceFilePath;

	LoadInputImage();
	inputImageWasLoaded = true;
	LoadReferenceFeatures(numberReferenceFilePath, numberKeypointsMatrix);
	numberReferenceWasLoaded = true;
	LoadReferenceFeatures(precisionReferenceFilePath, precisionKeypointsMatrix);
	precisionReferenceWasLoaded = true;
	}

void SelectionTester::ExecuteDfn()
	{
	dfn->imageInput(inputFrame);
	dfn->process();

	DELETE_IF_NOT_NULL(outputFeaturesVector);
	outputFeaturesVector = dfn->featuresSetOutput();

	PRINT_TO_LOG("Number of keypoints extracted is", GetNumberOfPoints(*outputFeaturesVector));
	}

bool SelectionTester::IsSelectionValid(float numberPercentageThreshold, unsigned pixelOutlierThreshold, float outliersPercentageThreshold)
	{
	ASSERT(inputImageWasLoaded && numberReferenceWasLoaded && precisionReferenceWasLoaded, "Error: some inputs were not correctly loaded");

	PRINT_TO_LOG("Numerosity Image Keypoints are", numberKeypointsMatrix.rows);
	PRINT_TO_LOG("Precision Image Keypoints are", precisionKeypointsMatrix.rows);

	bool numberOfKeypointsIsValid = ValidateNumberOfKeypoints(numberPercentageThreshold);
	bool keypointsAreValid = ValidateKeypoints(pixelOutlierThreshold, outliersPercentageThreshold);

	if (!numberOfKeypointsIsValid)
		{
		PRINT_TO_LOG("Number of keypoint is not valid according to reference numerosity", "");
		}
	if (!keypointsAreValid)
		{
		PRINT_TO_LOG("Keypoints are not valid according reference admissible keypoints", "");
		}

	return (numberOfKeypointsIsValid && keypointsAreValid);
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
	}

void SelectionTester::LoadInputImage()
	{
	cv::Mat cvImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(inputFrame);	
	inputFrame = frameConverter.Convert(cvImage);
	}

void SelectionTester::LoadReferenceFeatures(std::string& filePath, cv::Mat& keypointsMatrix)
	{
	cv::FileStorage opencvFile(filePath, cv::FileStorage::READ);
	opencvFile["KeypointsMatrix"] >> keypointsMatrix;
	opencvFile.release();
	ASSERT(keypointsMatrix.rows > 0 && keypointsMatrix.cols == 2 && keypointsMatrix.type() == CV_16UC1, "Error: reference keypoints are invalid");
	}

void SelectionTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

bool SelectionTester::ValidateNumberOfKeypoints(float numberPercentageThreshold)
	{
	float expectedNumberOfKeypoints = numberKeypointsMatrix.rows;
	float actualNumberOfKeypoints = GetNumberOfPoints(*outputFeaturesVector);
	float minNumberOfKeypoints = expectedNumberOfKeypoints * (1 - numberPercentageThreshold);
	float maxNumberOfKeypoints = expectedNumberOfKeypoints * (1 + numberPercentageThreshold);

	PRINT_TO_LOG("keypoints number: ", actualNumberOfKeypoints);
	return (actualNumberOfKeypoints >= minNumberOfKeypoints && actualNumberOfKeypoints <= maxNumberOfKeypoints);
	}

bool SelectionTester::ValidateKeypoints(unsigned pixelOutlierThreshold, float outliersPercentageThreshold)
	{
	int numberOfKeypoints = GetNumberOfPoints(*outputFeaturesVector);
	int numberOfValidKeypoints = 0;

	for(unsigned keypointIndex = 0; keypointIndex < numberOfKeypoints; keypointIndex++)
		{
		bool pointIsValid = false;
		for(unsigned referenceIndex = 0; referenceIndex < precisionKeypointsMatrix.rows && !pointIsValid; referenceIndex++)
			{
			unsigned pixelDistance = ComputePixelDistance(keypointIndex, referenceIndex);
			if (pixelDistance <= pixelOutlierThreshold)
				{
				numberOfValidKeypoints++;
				pointIsValid = true;
				}
			}
		}

	float percentageOfValidKeypoints = (float)numberOfValidKeypoints / (float)numberOfKeypoints;
	PRINT_TO_LOG("Percentage of valid keypoints: ", percentageOfValidKeypoints);
	return (percentageOfValidKeypoints >= outliersPercentageThreshold);
	}

unsigned SelectionTester::ComputePixelDistance(unsigned outputKeypointIndex, unsigned precisionReferenceKeypointIndex)
	{
	float x1 = GetXCoordinate(*outputFeaturesVector, outputKeypointIndex);
	float y1 = GetYCoordinate(*outputFeaturesVector, outputKeypointIndex);
	float x2 = precisionKeypointsMatrix.at<uint16_t>(precisionReferenceKeypointIndex, 0);
	float y2 = precisionKeypointsMatrix.at<uint16_t>(precisionReferenceKeypointIndex, 1);

	float distanceX = std::abs(x1-x2);
	float distanceY = std::abs(y1-y2);
	return (distanceX + distanceY);
	}

/** @} */
