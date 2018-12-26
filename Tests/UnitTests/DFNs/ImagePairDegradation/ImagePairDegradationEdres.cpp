/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImagePairDegradationEdres.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN ImagePairDegradationEdres.
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
#include <ImagePairDegradation/ImagePairDegradationEdres.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ImagePairDegradationEdres)", "[process]" )
{
    // Prepare input data
    cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRawLeft.png", cv::IMREAD_GRAYSCALE);
    cv::Mat inputImageRight = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRawRight.png", cv::IMREAD_GRAYSCALE);

    // Initialise a frame and set its metadata
    asn1SccFramePair *inputFramePair = new asn1SccFramePair();
    asn1SccFramePair_Initialize(inputFramePair);

    // Fill the metadata
    inputFramePair->msgVersion = frame_Version;

    // Left frame
    {
        inputFramePair->left.msgVersion = frame_Version;
        inputFramePair->left.metadata.msgVersion = frame_Version;
        inputFramePair->left.metadata.status = asn1Sccstatus_VALID;
        inputFramePair->left.metadata.pixelModel = asn1Sccpix_UNDEF;
        inputFramePair->left.metadata.mode = asn1Sccmode_GRAY;

        inputFramePair->left.data.msgVersion = array3D_Version;
        inputFramePair->left.data.rows = static_cast<asn1SccT_UInt32>(inputImageLeft.rows);
        inputFramePair->left.data.cols = static_cast<asn1SccT_UInt32>(inputImageLeft.cols);
        inputFramePair->left.data.channels = static_cast<asn1SccT_UInt32>(inputImageLeft.channels());
        inputFramePair->left.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageLeft.depth());
        inputFramePair->left.data.rowSize = inputImageLeft.step[0];
        inputFramePair->left.data.data.nCount = static_cast<int>(inputFramePair->left.data.rows * inputFramePair->left.data.rowSize);
        memcpy(inputFramePair->left.data.data.arr, inputImageLeft.data, static_cast<size_t>(inputFramePair->left.data.data.nCount));
    }

    // Right frame
    {
        inputFramePair->right.msgVersion = frame_Version;
        inputFramePair->right.metadata.msgVersion = frame_Version;
        inputFramePair->right.metadata.status = asn1Sccstatus_VALID;
        inputFramePair->right.metadata.pixelModel = asn1Sccpix_UNDEF;
        inputFramePair->right.metadata.mode = asn1Sccmode_GRAY;

        inputFramePair->right.data.msgVersion = array3D_Version;
        inputFramePair->right.data.rows = static_cast<asn1SccT_UInt32>(inputImageRight.rows);
        inputFramePair->right.data.cols = static_cast<asn1SccT_UInt32>(inputImageRight.cols);
        inputFramePair->right.data.channels = static_cast<asn1SccT_UInt32>(inputImageRight.channels());
        inputFramePair->right.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageRight.depth());
        inputFramePair->right.data.rowSize = inputImageRight.step[0];
        inputFramePair->right.data.data.nCount = static_cast<int>(inputFramePair->right.data.rows * inputFramePair->right.data.rowSize);
        memcpy(inputFramePair->right.data.data.arr, inputImageRight.data, static_cast<size_t>(inputFramePair->right.data.data.nCount));
    }

    // Instantiate DFN
    CDFF::DFN::ImagePairDegradation::ImagePairDegradationEdres* degradation = new CDFF::DFN::ImagePairDegradation::ImagePairDegradationEdres();

    // Send input data to DFN
    degradation->originalImagePairInput(*inputFramePair);

    // Run DFN
    degradation->process();

    // Query output data from DFN
    const asn1SccFramePair& output = degradation->degradedImagePairOutput();

    REQUIRE( output.left.data.msgVersion == array3D_Version );
    REQUIRE( output.left.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->left.data.rows) / static_cast<double>(degradation->parameters.yratio))) );
    REQUIRE( output.left.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->left.data.cols) / static_cast<double>(degradation->parameters.xratio))) );
    REQUIRE( output.left.data.channels == 1 );
    REQUIRE( output.left.data.depth == asn1Sccdepth_8U );
    REQUIRE( output.left.data.rowSize == output.left.data.cols );
    REQUIRE( output.left.data.data.nCount == static_cast<int>(output.left.data.rowSize * output.left.data.rows) );

    REQUIRE( output.right.data.msgVersion == array3D_Version );
    REQUIRE( output.right.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->right.data.rows) / static_cast<double>(degradation->parameters.yratio))) );
    REQUIRE( output.right.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFramePair->right.data.cols) / static_cast<double>(degradation->parameters.xratio))) );
    REQUIRE( output.right.data.channels == 1 );
    REQUIRE( output.right.data.depth == asn1Sccdepth_8U );
    REQUIRE( output.right.data.rowSize == output.right.data.cols );
    REQUIRE( output.right.data.data.nCount == static_cast<int>(output.right.data.rowSize * output.right.data.rows) );

    // Cleanup
    delete(degradation);
    delete(inputFramePair);
}


/** @} */
