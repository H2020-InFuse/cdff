/**
 * @author Nassir W. Oumer
 */

/**
 * Unit tests for the DFN CannyEdgeDetection
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <ImageFiltering/CannyEdgeDetection.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (Canny)", "[process]" )
{
	// Prepare input data
	cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Mat gray;
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	const Frame* input = MatToFrameConverter().Convert(gray);

	// Instantiate DFN
	CannyEdgeDetection* edgeDetection = new CannyEdgeDetection;

	// Send input data to DFN
	edgeDetection->imageInput(*input);

	// Run DFN
	edgeDetection->process();

	// Query output data from DFN
	const Frame& outputX = edgeDetection->imageOutput();

	// Cleanup
	delete edgeDetection;
	delete input;
}

TEST_CASE( "Call to configure (Canny) ", "[configure]" )
{
	// Instantiate DFN
	CannyEdgeDetection* edgeDetection = new CannyEdgeDetection;

	// Setup DFN
	edgeDetection->setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/CannyEdgeDetection_Conf.yaml");
	edgeDetection->configure();

	// Cleanup
	delete edgeDetection;
}

/** @} */
