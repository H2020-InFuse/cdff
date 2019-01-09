/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ColorConversion.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN ColorConversion.
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
#include <ColorConversion/ColorConversion.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ColorConversion)", "[process]" )
{
    // Prepare input data
    cv::Mat inputImage = cv::imread("../tests/Data/Images/LabChairLeft.png", cv::IMREAD_COLOR);

    // Initialise a frame and set its metadata
    asn1SccFrame *inputFrame = new asn1SccFrame();
    asn1SccFrame_Initialize(inputFrame);

    // Fill the metadata
    inputFrame->msgVersion = frame_Version;

    inputFrame->metadata.msgVersion = frame_Version;
    inputFrame->metadata.status = asn1Sccstatus_VALID;
    inputFrame->metadata.pixelModel = asn1Sccpix_UNDEF;
    inputFrame->metadata.mode = asn1Sccmode_BGR;

    inputFrame->data.msgVersion = array3D_Version;
    inputFrame->data.rows = static_cast<asn1SccT_UInt32>(inputImage.rows);
    inputFrame->data.cols = static_cast<asn1SccT_UInt32>(inputImage.cols);
    inputFrame->data.channels = static_cast<asn1SccT_UInt32>(inputImage.channels());
    inputFrame->data.depth = static_cast<asn1SccArray3D_depth_t>(inputImage.depth());
    inputFrame->data.rowSize = inputImage.step[0];
    inputFrame->data.data.nCount = static_cast<int>(inputFrame->data.rows * inputFrame->data.rowSize);
    memcpy(inputFrame->data.data.arr, inputImage.data, static_cast<size_t>(inputFrame->data.data.nCount));

    // Instantiate DFN
    CDFF::DFN::ColorConversion::ColorConversion* conversion = new CDFF::DFN::ColorConversion::ColorConversion();

    // Send input data to DFN
    conversion->originalImageInput(*inputFrame);

    // Run DFN
    for(int targetMode = 1; targetMode < asn1Sccmode_YCrCb; targetMode ++){
        if(targetMode == asn1Sccmode_UYVY)
            continue;

        conversion->parameters.targetMode = targetMode;

        conversion->process();

        // Query output data from DFN
        const asn1SccFrame& output = conversion->convertedImageOutput();

        REQUIRE( output.metadata.mode == static_cast<asn1SccFrame_mode_t>(conversion->parameters.targetMode) );
        REQUIRE( output.data.msgVersion == array3D_Version );
        REQUIRE( output.data.rows == inputFrame->data.rows);
        REQUIRE( output.data.cols == inputFrame->data.cols);
        REQUIRE( output.data.channels == (output.metadata.mode == asn1Sccmode_GRAY ? 1 : ((output.metadata.mode == asn1Sccmode_RGBA || output.metadata.mode == asn1Sccmode_BGRA) ? 4 : 3)) );
        REQUIRE( output.data.depth == asn1Sccdepth_8U );
        REQUIRE( output.data.rowSize == output.data.cols * output.data.channels );
        REQUIRE( output.data.data.nCount == static_cast<int>(output.data.rowSize * output.data.rows) );
    }

    // Cleanup
    delete(conversion);
    delete(inputFrame);
}


/** @} */
