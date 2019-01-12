/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageDegradation.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN ImageDegradation.
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
#include <ImageDegradation/ImageDegradation.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ImageDegradation)", "[process]" )
{
    // Prepare input data
    cv::Mat inputImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRawLeft.png", cv::IMREAD_GRAYSCALE);

    // Initialise a frame and set its metadata
    asn1SccFrame *inputFrame = new asn1SccFrame();
    asn1SccFrame_Initialize(inputFrame);

    // Fill the metadata
    inputFrame->msgVersion = frame_Version;

    inputFrame->metadata.msgVersion = frame_Version;
    inputFrame->metadata.status = asn1Sccstatus_VALID;
    inputFrame->metadata.pixelModel = asn1Sccpix_UNDEF;
    inputFrame->metadata.mode = asn1Sccmode_GRAY;

    inputFrame->data.msgVersion = array3D_Version;
    inputFrame->data.rows = static_cast<asn1SccT_UInt32>(inputImage.rows);
    inputFrame->data.cols = static_cast<asn1SccT_UInt32>(inputImage.cols);
    inputFrame->data.channels = static_cast<asn1SccT_UInt32>(inputImage.channels());
    inputFrame->data.depth = static_cast<asn1SccArray3D_depth_t>(inputImage.depth());
    inputFrame->data.rowSize = inputImage.step[0];
    inputFrame->data.data.nCount = static_cast<int>(inputFrame->data.rows * inputFrame->data.rowSize);
    memcpy(inputFrame->data.data.arr, inputImage.data, static_cast<size_t>(inputFrame->data.data.nCount));

    // Instantiate DFN
    CDFF::DFN::ImageDegradation::ImageDegradation* degradation = new CDFF::DFN::ImageDegradation::ImageDegradation();

    // Send input data to DFN
    degradation->originalImageInput(*inputFrame);

    // Run DFN
    degradation->process();

    // Query output data from DFN
    const asn1SccFrame& output = degradation->degradedImageOutput();

    REQUIRE( output.data.msgVersion == array3D_Version );
    REQUIRE( output.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFrame->data.rows) / static_cast<double>(degradation->parameters.yratio))) );
    REQUIRE( output.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFrame->data.cols) / static_cast<double>(degradation->parameters.xratio))) );
    REQUIRE( output.data.channels == 1 );
    REQUIRE( output.data.depth == asn1Sccdepth_8U );
    REQUIRE( output.data.rowSize == output.data.cols );
    REQUIRE( output.data.data.nCount == static_cast<int>(output.data.rowSize * output.data.rows) );

    // Cleanup
    delete(degradation);
    delete(inputFrame);
}


/** @} */
