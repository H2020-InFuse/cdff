/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DepthFiltering.cpp
 * @date 04/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Depth Filtering.
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
#include <DepthFiltering/ConvolutionFilter.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Depth Filtering)", "[process]" )
{
	// Prepare input data
    cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
	Converters::MatToFrameConverter matToFrame;
	FrameWrapper::FrameConstPtr inputFrame = matToFrame.Convert(inputImage);

	// Instantiate DFN
	CDFF::DFN::DepthFiltering::ConvolutionFilter* filter = new CDFF::DFN::DepthFiltering::ConvolutionFilter();

	// Send input data to DFN
	filter->frameInput(*inputFrame);

	// Run DFN
	filter->process();

	// Query output data from DFN
	const asn1SccFrame& output = filter->frameOutput();

	CHECK( output.data.depth == asn1Sccdepth_8U );
	CHECK( output.data.data.nCount > 0 );

	// Cleanup
	delete(filter);
}


/** @} */
