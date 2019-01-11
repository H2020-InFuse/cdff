/**
 * Unit tests for the DFN BackgroundExtraction
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <ImageFiltering/BackgroundExtraction.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (BackgroundExtraction)", "[process]" )
{
	// Prepare input data
	cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Mat gray;
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	FrameSharedConstPtr input = MatToFrameConverter().ConvertShared(gray);

	// Instantiate DFN
	std::unique_ptr<BackgroundExtraction> background_subtractor (new BackgroundExtraction);

	// Send input data to DFN
	background_subtractor->imageInput(*input);

	// Run DFN
	background_subtractor->process();

	// Query output data from DFN
	const Frame& output = background_subtractor->imageOutput();

	// Check the DFN output image is not empty
	BOOST_ASSERT(output.data.data.nCount > 0);
}

TEST_CASE( "Call to configure (BackgroundExtraction) ", "[configure]" )
{
	// Prepare input data
	cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Mat gray;
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	FrameSharedConstPtr input = MatToFrameConverter().ConvertShared(gray);

	// Instantiate DFN
	std::unique_ptr<BackgroundExtraction> background_subtractor (new BackgroundExtraction);

	// Setup DFN
	background_subtractor->setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/BackgroundExtraction.yaml");
	background_subtractor->configure();

	// Send input data to DFN
	background_subtractor->imageInput(*input);

	// Run DFN
	background_subtractor->process();

	// Query output data from DFN
	const Frame& output = background_subtractor->imageOutput();

	// Check the DFN output image is not empty
	BOOST_ASSERT(output.data.data.nCount > 0);
}

/** @} */
