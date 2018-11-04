/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN DisparityMapping
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <StereoReconstruction/DisparityMapping.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace CDFF::DFN::StereoReconstruction;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace FrameWrapper;

TEST_CASE( "DFN DisparityMapping: processing step", "[process]" )
{
	// Prepare input data
	cv::Mat stereoImage = cv::imread("../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cv::Mat leftImage = stereoImage(cv::Rect(0, 0, stereoImage.cols/2, stereoImage.rows));
	cv::Mat rightImage = stereoImage(cv::Rect(stereoImage.cols/2, 0, stereoImage.cols/2, stereoImage.rows));

	MatToFrameConverter matToFrame;
	const Frame* left = matToFrame.Convert(leftImage);
	const Frame* right = matToFrame.Convert(rightImage);

	// Instantiate DFN
	DisparityMapping* disparityMapping = new DisparityMapping;

	// Send input data to DFN
	disparityMapping->leftInput(*left);
	disparityMapping->rightInput(*right);

	// Run DFN
	disparityMapping->process();

	// Query output data from DFN
	const PointCloud& reconstructedScene = disparityMapping->pointcloudOutput();

	// Cleanup
	delete disparityMapping;
	delete left;
	delete right;
}

TEST_CASE( "DFN DisparityMapping: configuration", "[configure]" )
{
	// Instantiate DFN
	DisparityMapping* disparityMapping = new DisparityMapping;

	// Setup DFN
	disparityMapping->setConfigurationFile("../tests/ConfigurationFiles/DFNs/StereoReconstruction/DisparityMapping_Conf1.yaml");
	disparityMapping->configure();
}

/** @} */
