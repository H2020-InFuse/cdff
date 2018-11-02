/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortion.cpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN ImageUndistortion.
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
#include <ImageFiltering/ImageUndistortion.hpp>
#include <Converters/MatToFrameConverter.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace Converters;
using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (image undistortion)", "[process]" )
{
	// Prepare input data
	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
	FrameConstPtr inputFrame = MatToFrameConverter().Convert(inputImage);

	// Instantiate DFN
	ImageUndistortion* filter = new ImageUndistortion;

	// Send input data to DFN
	filter->imageInput(*inputFrame);

	// Run DFN
	filter->process();

	// Query output data from DFN
	const Frame& output = filter->imageOutput();

	// Cleanup
	delete filter;
}

TEST_CASE( "Call to configure (image undistortion)", "[configure]" )
{
	// Instantiate DFN
	ImageUndistortion* filter = new ImageUndistortion;

	// Setup DFN
	filter->setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortion_Conf1.yaml");
	filter->configure();

	// Cleanup
	delete filter;
}

/** @} */
