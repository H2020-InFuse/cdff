/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FlannMatcher.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN FlannMatcher.
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
#include <catch.hpp>

#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (FLANN registration)", "[process]" )
{
	// Previous input data
	/*
	const unsigned NUMBER_OF_POINTS = 10;
	const unsigned DESCRIPTOR_LENGTH = 8;
	cv::Mat inputImage(NUMBER_OF_POINTS, DESCRIPTOR_LENGTH+2, CV_32FC1, cv::Scalar(0));
	for (unsigned pointIndex = 0; pointIndex < NUMBER_OF_POINTS; pointIndex++)
	{
		inputImage.at<float>(pointIndex, 0) = 10;
		inputImage.at<float>(pointIndex, 1) = (float)pointIndex + 10;
		for (unsigned componentIndex = 0; componentIndex < DESCRIPTOR_LENGTH; componentIndex++)
		{
			inputImage.at<float>(pointIndex, componentIndex+2) = (float)pointIndex + (float)componentIndex / 100;
		}
	}
	*/

	// First run a feature extraction/description DFN on actual data
	// to prepare input data for the feature matching DFN

	// Prepare input data
	cv::Mat doubleImage = cv::imread("../tests/Data/Images/DevonIslandLeft.ppm", cv::IMREAD_COLOR);
	cv::Mat inputImage = doubleImage( cv::Rect(0, 0, doubleImage.cols/2, doubleImage.rows) );
	FrameConstPtr inputFrame = MatToFrameConverter().Convert(inputImage);

	// Instantiate DFN
	OrbDetectorDescriptor* orb = new OrbDetectorDescriptor;

	// Send input data to DFN
	orb->frameInput(*inputFrame);

	// Setup DFN
	orb->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription2D/OrbDescriptor_Conf1.yaml");
	orb->configure();

	// Run DFN
	orb->process();

	// Now test the feature matching DFN

	// Prepare input data
	const VisualPointFeatureVector2D& inputFeaturesA = orb->featuresOutput();
	const VisualPointFeatureVector2D& inputFeaturesB = orb->featuresOutput();

	// Instantiate DFN
	FlannMatcher* flann = new FlannMatcher;

	// Send input data to DFN
	flann->sourceFeaturesInput(inputFeaturesA);
	flann->sinkFeaturesInput(inputFeaturesB);

	// Run DFN
	flann->process();

	// Query output data from DFN
	const CorrespondenceMap2D& output = flann->matchesOutput();

	// Cleanup
	delete orb;
	delete flann;
}

TEST_CASE( "Call to configure (FLANN registration)", "[configure]" )
{
	// Instantiate DFN
	FlannMatcher* flann = new FlannMatcher;

	// Setup DFN
	flann->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching2D/FlannMatcher_Conf1.yaml");
	flann->configure();

	// Cleanup
	delete flann;
}

/** @} */
