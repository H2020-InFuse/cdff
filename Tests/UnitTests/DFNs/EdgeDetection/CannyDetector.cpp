/**
 * @author Nassir W. Oumer
 */

/**
 * Unit tests for the DFN CannyDetector
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <EdgeDetection/CannyDetector.hpp>
#include <MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace dfn_ci;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (Canny edge detector)", "[process]" )
{
	// Prepare input data
	cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Mat gray;
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	MatToFrameConverter converter;
	const Frame* input = converter.Convert(gray);

	// Instantiate DFN
	CannyDetector* detector = new CannyDetector;

	// Send input data to DFN
	detector->imageInput(*input);

	// Run DFN
	detector->process();

	// Query output data from DFN
	const Frame& output = detector->edgeMapOutput();

	// Cleanup
	delete detector;
	delete input;
}

TEST_CASE( "Call to configure (Canny edge detector)", "[configure]" )
{
	// Instantiate DFN
	CannyDetector* detector = new CannyDetector;

	// Setup DFN
	detector->setConfigurationFile("../tests/ConfigurationFiles/DFNs/EdgeDetection/CannyDetector_Conf.yaml");
	detector->configure();

	// Cleanup
	delete detector;
}

/** @} */
