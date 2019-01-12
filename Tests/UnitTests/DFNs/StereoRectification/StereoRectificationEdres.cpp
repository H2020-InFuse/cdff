/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoRectificationEdres.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN StereoRectificationEdres.
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
#include <StereoRectification/StereoRectificationEdres.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (StereoRectificationEdres)", "[process]" )
{
    // Prepare input data
    cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/EdresImages/NavL.png", cv::IMREAD_GRAYSCALE);
    cv::Mat inputImageRight = cv::imread("../tests/Data/Images/EdresImages/NavR.png", cv::IMREAD_GRAYSCALE);

    // Initialise a frame and set its metadata
    asn1SccFramePair *inputFramePair = new asn1SccFramePair();
    asn1SccFramePair_Initialize(inputFramePair);

    // Fill the metadata
    inputFramePair->msgVersion = frame_Version;

    // Left frame
    {
        asn1SccFrame& inputFrame = inputFramePair->left;
        inputFrame.msgVersion = frame_Version;

        inputFrame.metadata.msgVersion = frame_Version;
        inputFrame.metadata.status = asn1Sccstatus_VALID;
        inputFrame.metadata.pixelModel = asn1Sccpix_UNDEF;
        inputFrame.metadata.mode = asn1Sccmode_GRAY;

        inputFrame.data.msgVersion = array3D_Version;
        inputFrame.data.rows = static_cast<asn1SccT_UInt32>(inputImageLeft.rows);
        inputFrame.data.cols = static_cast<asn1SccT_UInt32>(inputImageLeft.cols);
        inputFrame.data.channels = static_cast<asn1SccT_UInt32>(inputImageLeft.channels());
        inputFrame.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageLeft.depth());
        inputFrame.data.rowSize = inputImageLeft.step[0];
        inputFrame.data.data.nCount = static_cast<int>(inputFrame.data.rows * inputFrame.data.rowSize);
        memcpy(inputFrame.data.data.arr, inputImageLeft.data, static_cast<size_t>(inputFrame.data.data.nCount));
    }

    // Right frame
    {
        asn1SccFrame& inputFrame = inputFramePair->right;
        inputFrame.msgVersion = frame_Version;

        inputFrame.metadata.msgVersion = frame_Version;
        inputFrame.metadata.status = asn1Sccstatus_VALID;
        inputFrame.metadata.pixelModel = asn1Sccpix_UNDEF;
        inputFrame.metadata.mode = asn1Sccmode_GRAY;

        inputFrame.data.msgVersion = array3D_Version;
        inputFrame.data.rows = static_cast<asn1SccT_UInt32>(inputImageRight.rows);
        inputFrame.data.cols = static_cast<asn1SccT_UInt32>(inputImageRight.cols);
        inputFrame.data.channels = static_cast<asn1SccT_UInt32>(inputImageRight.channels());
        inputFrame.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageRight.depth());
        inputFrame.data.rowSize = inputImageRight.step[0];
        inputFrame.data.data.nCount = static_cast<int>(inputFrame.data.rows * inputFrame.data.rowSize);
        memcpy(inputFrame.data.data.arr, inputImageRight.data, static_cast<size_t>(inputFrame.data.data.nCount));
    }

    // Instantiate DFN
    CDFF::DFN::StereoRectification::StereoRectificationEdres* rectification = new CDFF::DFN::StereoRectification::StereoRectificationEdres();
    rectification->parameters.mapFileLeft = "../tests/Data/Images/EdresImages/camL";
    rectification->parameters.mapFileRight = "../tests/Data/Images/EdresImages/camR";
    rectification->parameters.xratio = 2;
    rectification->parameters.yratio = 2;

    // Send input data to DFN
    rectification->originalStereoPairInput(*inputFramePair);

    // Run DFN
    rectification->process();

    // Query output data from DFN
    const asn1SccFramePair& output = rectification->rectifiedStereoPairOutput();

    // Check left
    {
        REQUIRE( output.left.data.msgVersion == array3D_Version );
        REQUIRE( output.left.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->left.data.rows) / static_cast<double>(rectification->parameters.yratio))) );
        REQUIRE( output.left.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->left.data.cols) / static_cast<double>(rectification->parameters.xratio))) );
        REQUIRE( output.left.data.channels == 1 );
        REQUIRE( output.left.data.depth == asn1Sccdepth_8U );
        REQUIRE( output.left.data.rowSize == output.left.data.cols );
        REQUIRE( output.left.data.data.nCount == static_cast<int>(output.left.data.rowSize * output.left.data.rows) );
        REQUIRE( output.left.intrinsic.cameraModel == asn1Scccam_MAPS );

        for(int i = 0; i < output.left.intrinsic.distCoeffs.nCount; i ++){
            REQUIRE( output.left.intrinsic.distCoeffs.arr[0] == 0 );
        }
    }

    // Check right
    {
        REQUIRE( output.right.data.msgVersion == array3D_Version );
        REQUIRE( output.right.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->right.data.rows) / static_cast<double>(rectification->parameters.yratio))) );
        REQUIRE( output.right.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->right.data.cols) / static_cast<double>(rectification->parameters.xratio))) );
        REQUIRE( output.right.data.channels == 1 );
        REQUIRE( output.right.data.depth == asn1Sccdepth_8U );
        REQUIRE( output.right.data.rowSize == output.right.data.cols );
        REQUIRE( output.right.data.data.nCount == static_cast<int>(output.right.data.rowSize * output.right.data.rows) );
        REQUIRE( output.right.intrinsic.cameraModel == asn1Scccam_MAPS );

        for(int i = 0; i < output.right.intrinsic.distCoeffs.nCount; i ++){
            REQUIRE( output.right.intrinsic.distCoeffs.arr[0] == 0 );
        }
    }

    // Cleanup
    delete(rectification);
    delete(inputFramePair);
}


/** @} */
