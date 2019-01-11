/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityFilteringEdres.cpp
 * @date 07/01/2019
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN DisparityFilteringEdres.
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
#include <DisparityFiltering/DisparityFilteringEdres.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (DisparityFilteringEdres)", "[process]" )
{
    // Prepare input data
    cv::Mat inputImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieDisparityDeg3Edres.exr", cv::IMREAD_ANYDEPTH);

    // Initialise a frame and set its metadata
    asn1SccFrame *inputFrame = new asn1SccFrame();
    asn1SccFrame_Initialize(inputFrame);

    // Fill the metadata
    inputFrame->msgVersion = frame_Version;

    inputFrame->metadata.msgVersion = frame_Version;
    inputFrame->metadata.status = asn1Sccstatus_VALID;

    inputFrame->metadata.pixelModel = asn1Sccpix_DISP;
    inputFrame->metadata.pixelCoeffs.arr[0] = 1.;
    inputFrame->metadata.pixelCoeffs.arr[1] = 0.;
    inputFrame->metadata.pixelCoeffs.arr[2] = 0.270282;
    inputFrame->metadata.pixelCoeffs.arr[3] = 3;
    inputFrame->metadata.pixelCoeffs.arr[4] = 184;

    inputFrame->metadata.mode = asn1Sccmode_UNDEF;

    inputFrame->data.msgVersion = array3D_Version;
    inputFrame->data.rows = static_cast<asn1SccT_UInt32>(inputImage.rows);
    inputFrame->data.cols = static_cast<asn1SccT_UInt32>(inputImage.cols);
    inputFrame->data.channels = static_cast<asn1SccT_UInt32>(inputImage.channels());
    inputFrame->data.depth = static_cast<asn1SccArray3D_depth_t>(inputImage.depth());
    inputFrame->data.rowSize = inputImage.step[0];
    inputFrame->data.data.nCount = static_cast<int>(inputFrame->data.rows * inputFrame->data.rowSize);
    memcpy(inputFrame->data.data.arr, inputImage.data, static_cast<size_t>(inputFrame->data.data.nCount));

    // Instantiate DFN
    CDFF::DFN::DisparityFiltering::DisparityFilteringEdres* filter = new CDFF::DFN::DisparityFiltering::DisparityFilteringEdres();

    // Send input data to DFN
    filter->rawDisparityInput(*inputFrame);

    // Run DFN
    filter->process();

    // Query output data from DFN
    const asn1SccFrame& output = filter->filteredDisparityOutput();

    REQUIRE( output.metadata.pixelModel == asn1Sccpix_DISP );
    REQUIRE( output.metadata.pixelCoeffs.arr[2] == inputFrame->metadata.pixelCoeffs.arr[2] );
    REQUIRE( output.data.msgVersion == array3D_Version );
    REQUIRE( output.data.rows == inputFrame->data.rows );
    REQUIRE( output.data.cols == inputFrame->data.cols );
    REQUIRE( output.data.channels == 1 );
    REQUIRE( output.data.depth == inputFrame->data.depth );
    REQUIRE( output.data.rowSize == inputFrame->data.rowSize );
    REQUIRE( output.data.data.nCount == static_cast<int>(output.data.rowSize * output.data.rows) );

    // Cleanup
    delete(filter);
    delete(inputFrame);
}


/** @} */
