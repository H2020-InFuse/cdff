/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityToPointCloudEdres.cpp
 * @date 20/12/2018
 * @author Clément Bazerque
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Disparity To PointCloud Edres.
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
#include <DisparityToPointCloud/DisparityToPointCloudEdres.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Disparity To PointCloud Edres)", "[process]" )
{
	// Loads a disparity image
	cv::Mat cvDispImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRawDisparity.exr", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

	// Convert it to a frame
	asn1SccFrame *dispImage = new asn1SccFrame();
	asn1SccFrame_Initialize(dispImage);
	dispImage->msgVersion = frame_Version;
	dispImage->metadata.msgVersion = frame_Version;
	dispImage->metadata.status = asn1Sccstatus_VALID;
	dispImage->metadata.pixelModel = asn1Sccpix_DISP;
	dispImage->metadata.mode = asn1Sccmode_GRAY;

	dispImage->intrinsic.msgVersion = frame_Version;
	dispImage->intrinsic.cameraModel = asn1Scccam_PINHOLE;
	dispImage->intrinsic.cameraMatrix.arr[0].arr[0] = 1069.23438117834;
	dispImage->intrinsic.cameraMatrix.arr[0].arr[1] = 0;
	dispImage->intrinsic.cameraMatrix.arr[0].arr[2] = 956.907250034732;
	dispImage->intrinsic.cameraMatrix.arr[1].arr[0] = 0;
	dispImage->intrinsic.cameraMatrix.arr[1].arr[1] = 1071.73940921359;
	dispImage->intrinsic.cameraMatrix.arr[1].arr[2] = 621.662860111553;
	dispImage->intrinsic.cameraMatrix.arr[2].arr[0] = 0;
	dispImage->intrinsic.cameraMatrix.arr[2].arr[1] = 0;
	dispImage->intrinsic.cameraMatrix.arr[2].arr[2] = 1;

	asn1SccArray3D &imageOnly = dispImage->data;
	imageOnly.msgVersion = array3D_Version;
	imageOnly.rows = static_cast<asn1SccT_UInt32>(cvDispImage.rows);
	imageOnly.cols = static_cast<asn1SccT_UInt32>(cvDispImage.cols);
	imageOnly.channels = static_cast<asn1SccT_UInt32>(cvDispImage.channels());
	imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvDispImage.depth());
	imageOnly.rowSize = cvDispImage.step[0];
	imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
	memcpy(imageOnly.data.arr, cvDispImage.data, static_cast<size_t>(imageOnly.data.nCount));

	// Instantiate DFN
	CDFF::DFN::DisparityToPointCloud::DisparityToPointCloudEdres *disparityToPointCloudEdres = new CDFF::DFN::DisparityToPointCloud::DisparityToPointCloudEdres();

	// Send input data to DFN
	disparityToPointCloudEdres->dispImageInput(*dispImage);

	// Run DFN
	disparityToPointCloudEdres->process();

	// Query output data from DFN
	const asn1SccPointcloud &output = disparityToPointCloudEdres->pointCloudOutput();

    REQUIRE(output.metadata.msgVersion == pointCloud_Version);
    REQUIRE(output.metadata.height == dispImage->data.rows);
    REQUIRE(output.metadata.width == dispImage->data.cols);
    REQUIRE(output.metadata.isOrdered == true);
    REQUIRE(output.data.points.nCount == dispImage->data.rows * dispImage->data.cols);

	// Cleanup
	delete(dispImage);
	delete(disparityToPointCloudEdres);
}


/** @} */
