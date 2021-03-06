/**
 * @author Nassir W. Oumer
 */

/**
 * Unit tests for the DFN DerivativeEdgeDetection
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <ImageFiltering/DerivativeEdgeDetection.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (Sobel derivatives)", "[process]" )
{
	// Prepare input data
	cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Mat gray;
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	const Frame* input = MatToFrameConverter().Convert(gray);

	// Instantiate DFN
	DerivativeEdgeDetection* sobelGradient = new DerivativeEdgeDetection;

	// Send input data to DFN
	sobelGradient->imageInput(*input);

	// Run DFN
	sobelGradient->process();

	// Query output data from DFN
	const Frame& outputX = sobelGradient->imageOutput();

	// Cleanup
	delete sobelGradient;
	delete input;
}

TEST_CASE( "Call to configure (Sobel derivatives) ", "[configure]" )
{
	// Instantiate DFN
	DerivativeEdgeDetection* sobelGradient = new DerivativeEdgeDetection;

	// Setup DFN
	sobelGradient->setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/DerivativeEdgeDetection_Conf.yaml");
	sobelGradient->configure();

	// Cleanup
	delete sobelGradient;
}

/** @} */
