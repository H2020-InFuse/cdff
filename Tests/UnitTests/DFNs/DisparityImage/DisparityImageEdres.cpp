/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityImageEdres.cpp
 * @date 20/12/2018
 * @author Clément Bazerque
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Disparity Image Edres.
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
#include <DisparityImage/DisparityImageEdres.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Disparity Image Edres)", "[process]" )
{
	// Loads two grayscaled rectified images
	cv::Mat cvLeftImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectLeft.png", cv::IMREAD_GRAYSCALE);
	cv::Mat cvRightImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectRight.png", cv::IMREAD_GRAYSCALE);

	// Initialise a frame pair and set its metadata
	asn1SccFramePair *framePair = new asn1SccFramePair();
	asn1SccFramePair_Initialize(framePair);
	framePair->msgVersion = frame_Version;
	framePair->baseline = 0.270268442641143;

	// Fill the left image data
	{
		framePair->left.msgVersion = frame_Version;
		framePair->left.metadata.msgVersion = frame_Version;
		framePair->left.metadata.status = asn1Sccstatus_VALID;
		framePair->left.metadata.pixelModel = asn1Sccpix_UNDEF;
		framePair->left.metadata.mode = asn1Sccmode_GRAY;

		framePair->left.intrinsic.msgVersion = frame_Version;
		framePair->left.intrinsic.cameraModel = asn1Scccam_PINHOLE;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[0] = 1069.23438117834;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[2] = 956.907250034732;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[1] = 1071.73940921359;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[2] = 621.662860111553;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

		asn1SccArray3D &imageOnly = framePair->left.data;
		imageOnly.msgVersion = array3D_Version;
		imageOnly.rows = static_cast<asn1SccT_UInt32>(cvLeftImage.rows);
		imageOnly.cols = static_cast<asn1SccT_UInt32>(cvLeftImage.cols);
		imageOnly.channels = static_cast<asn1SccT_UInt32>(cvLeftImage.channels());
		imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvLeftImage.depth());
		imageOnly.rowSize = cvLeftImage.step[0];
		imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
		memcpy(imageOnly.data.arr, cvLeftImage.data, static_cast<size_t>(imageOnly.data.nCount));
	}

	// Fill the right image data
	{
		framePair->right.msgVersion = frame_Version;
		framePair->right.metadata.msgVersion = frame_Version;
		framePair->right.metadata.status = asn1Sccstatus_VALID;
		framePair->right.metadata.pixelModel = asn1Sccpix_UNDEF;
		framePair->right.metadata.mode = asn1Sccmode_GRAY;

		framePair->right.intrinsic.msgVersion = frame_Version;
		framePair->right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[0] = 1066.44771376037;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[2] = 943.937535155249;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[1] = 1069.28469804725;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[2] = 617.141031157546;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[2] = 1;
		asn1SccArray3D &imageOnly = framePair->right.data;
		imageOnly.msgVersion = array3D_Version;
		imageOnly.rows = static_cast<asn1SccT_UInt32>(cvRightImage.rows);
		imageOnly.cols = static_cast<asn1SccT_UInt32>(cvRightImage.cols);
		imageOnly.channels = static_cast<asn1SccT_UInt32>(cvRightImage.channels());
		imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvRightImage.depth());
		imageOnly.rowSize = cvRightImage.step[0];
		imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
		memcpy(imageOnly.data.arr, cvRightImage.data, static_cast<size_t>(imageOnly.data.nCount));
	}

	// Instantiate DFN
	CDFF::DFN::DisparityImage::DisparityImageEdres *disparityImageEdres = new CDFF::DFN::DisparityImage::DisparityImageEdres();

	// Send input data to DFN
	disparityImageEdres->framePairInput(*framePair);

	// Run DFN
	disparityImageEdres->process();

	// Query output data from DFN
    const asn1SccFrame &output = disparityImageEdres->disparityOutput();

    REQUIRE( output.metadata.pixelModel == asn1Sccpix_DISP );
    REQUIRE( output.metadata.pixelCoeffs.arr[2] == framePair->baseline );
    REQUIRE( output.data.msgVersion == array3D_Version );
    REQUIRE( output.data.rows == framePair->left.data.rows );
    REQUIRE( output.data.cols == framePair->left.data.cols );
    REQUIRE( output.data.channels == 1 );
    REQUIRE( output.data.data.nCount == static_cast<int>(output.data.rowSize * output.data.rows) );

    // Cleanup
	delete(framePair);
	delete(disparityImageEdres);
}


/** @} */
