/**
 * @author Nassir W. Oumer
 */

/**
 * Unit tests for the DFN SobelDerivative
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <EdgeDetection/SobelDerivative.hpp>
#include <MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace dfn_ci;
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
	SobelDerivative* sobelGradient = new SobelDerivative;

	// Send input data to DFN
	sobelGradient->imageInput(*input);

	// Run DFN
	sobelGradient->process();

	// Query output data from DFN
	const Frame& outputX = sobelGradient->sobelGradientXOutput();
	const Frame& outputY = sobelGradient->sobelGradientYOutput();

	// Cleanup
	delete sobelGradient;
	delete input;
}

TEST_CASE( "Call to configure (Sobel derivatives) ", "[configure]" )
{
	// Instantiate DFN
	SobelDerivative* sobelGradient = new SobelDerivative;

	// Setup DFN
	sobelGradient->setConfigurationFile("../tests/ConfigurationFiles/DFNs/EdgeDetection/SobelDerivative_Conf.yaml");
	sobelGradient->configure();

	// Cleanup
	delete sobelGradient;
}

/** @} */
