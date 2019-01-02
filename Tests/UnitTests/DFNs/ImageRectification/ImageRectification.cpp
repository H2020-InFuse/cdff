/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageRectification.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN ImageRectification.
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
#include <ImageRectification/ImageRectification.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ImageRectification)", "[process]" )
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

    // Data taken from "CDFF/Tests/Data/Images/MinnieStereo/tisc-33UP2000_17810171-tisc-33UP2000_17810152.yml"
    inputFrame->intrinsic.cameraMatrix.arr[0].arr[0] = 1069.23438117834;
    inputFrame->intrinsic.cameraMatrix.arr[0].arr[1] = 0.;
    inputFrame->intrinsic.cameraMatrix.arr[0].arr[2] = 956.907250034732;
    inputFrame->intrinsic.cameraMatrix.arr[1].arr[0] = 0.;
    inputFrame->intrinsic.cameraMatrix.arr[1].arr[1] = 1071.73940921359;
    inputFrame->intrinsic.cameraMatrix.arr[1].arr[2] = 621.662860111553;
    inputFrame->intrinsic.cameraMatrix.arr[2].arr[0] = 0.;
    inputFrame->intrinsic.cameraMatrix.arr[2].arr[1] = 0.;
    inputFrame->intrinsic.cameraMatrix.arr[2].arr[2] = 1.;

    inputFrame->intrinsic.distCoeffs.arr[0] = -0.221269881741857;
    inputFrame->intrinsic.distCoeffs.arr[1] = 0.059249716294198;
    inputFrame->intrinsic.distCoeffs.arr[2] = -3.518067425067891e-04;
    inputFrame->intrinsic.distCoeffs.arr[3] = 1.460985115336601e-04;
    inputFrame->intrinsic.distCoeffs.nCount = 4;

    std::string sensorId = "tisc-33UP2000_17810171";
    inputFrame->intrinsic.sensorId.nCount = static_cast<int>(sensorId.size() +1);
    memcpy(inputFrame->intrinsic.sensorId.arr, sensorId.data(), static_cast<size_t>(inputFrame->intrinsic.sensorId.nCount));

    inputFrame->intrinsic.cameraModel = asn1Scccam_PINHOLE;
    inputFrame->intrinsic.msgVersion = frame_Version;

    inputFrame->data.msgVersion = array3D_Version;
    inputFrame->data.rows = static_cast<asn1SccT_UInt32>(inputImage.rows);
    inputFrame->data.cols = static_cast<asn1SccT_UInt32>(inputImage.cols);
    inputFrame->data.channels = static_cast<asn1SccT_UInt32>(inputImage.channels());
    inputFrame->data.depth = static_cast<asn1SccArray3D_depth_t>(inputImage.depth());
    inputFrame->data.rowSize = inputImage.step[0];
    inputFrame->data.data.nCount = static_cast<int>(inputFrame->data.rows * inputFrame->data.rowSize);
    memcpy(inputFrame->data.data.arr, inputImage.data, static_cast<size_t>(inputFrame->data.data.nCount));

    // Instantiate DFN
    CDFF::DFN::ImageRectification::ImageRectification* rectification = new CDFF::DFN::ImageRectification::ImageRectification();

    // Send input data to DFN
    rectification->originalImageInput(*inputFrame);

    // Run DFN
    rectification->process();

    // Query output data from DFN
    const asn1SccFrame& output = rectification->rectifiedImageOutput();

    REQUIRE( output.data.msgVersion == array3D_Version );
    REQUIRE( output.data.rows == static_cast<int>(std::ceil(static_cast<double>(inputFrame->data.rows) / static_cast<double>(rectification->parameters.yratio))) );
    REQUIRE( output.data.cols == static_cast<int>(std::ceil(static_cast<double>(inputFrame->data.cols) / static_cast<double>(rectification->parameters.xratio))) );
    REQUIRE( output.data.channels == 1 );
    REQUIRE( output.data.depth == asn1Sccdepth_8U );
    REQUIRE( output.data.rowSize == output.data.cols );
    REQUIRE( output.data.data.nCount == static_cast<int>(output.data.rowSize * output.data.rows) );

    if( rectification->parameters.fisheye ){
    REQUIRE( output.intrinsic.cameraModel == asn1Scccam_FISHEYE );
    }
    else{
        REQUIRE( output.intrinsic.cameraModel == asn1Scccam_PINHOLE );
    }

    REQUIRE( output.intrinsic.distCoeffs.nCount == inputFrame->intrinsic.distCoeffs.nCount );

    for(int i = 0; i < output.intrinsic.distCoeffs.nCount; i ++){
        REQUIRE( output.intrinsic.distCoeffs.arr[0] == 0 );
    }

    // Cleanup
    delete(rectification);
    delete(inputFrame);
}


/** @} */
