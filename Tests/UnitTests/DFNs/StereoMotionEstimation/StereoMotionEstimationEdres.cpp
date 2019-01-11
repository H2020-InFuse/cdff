/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoMotionEstimationEdres.cpp
 * @date 07/01/2019
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN StereoMotionEstimationEdres.
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
#include <StereoMotionEstimation/StereoMotionEstimationEdres.hpp>
#include <Types/C/Frame.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (StereoMotionEstimationEdres)", "[process]" )
{
    // Instantiate DFN
    CDFF::DFN::StereoMotionEstimation::StereoMotionEstimationEdres* vme = new CDFF::DFN::StereoMotionEstimation::StereoMotionEstimationEdres();

    vme->configure();

    asn1SccFramePair *inputFramePair = new asn1SccFramePair();
    asn1SccFrame *dispImage = new asn1SccFrame();

    // First Image
    {
        // Prepare input data
        cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectDeg3Left.png", cv::IMREAD_GRAYSCALE);
        cv::Mat inputImageRight = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectDeg3Right.png", cv::IMREAD_GRAYSCALE);

        // Initialise a frame and set its metadata
        asn1SccFramePair_Initialize(inputFramePair);

        // Fill the metadata
        inputFramePair->msgVersion = frame_Version;
        inputFramePair->baseline = -0.270268442641143;

        // Left frame
        {
            inputFramePair->left.msgVersion = frame_Version;
            inputFramePair->left.metadata.msgVersion = frame_Version;
            inputFramePair->left.metadata.status = asn1Sccstatus_VALID;
            inputFramePair->left.metadata.pixelModel = asn1Sccpix_UNDEF;
            inputFramePair->left.metadata.mode = asn1Sccmode_GRAY;

            inputFramePair->left.intrinsic.msgVersion = frame_Version;
            inputFramePair->left.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            inputFramePair->left.extrinsic.hasFixedTransform = true;

            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00173265;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0115965;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322039;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946654;

            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.2624;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.0862;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27392;

            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.295922;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = 0.124542;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66978;

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

            inputFramePair->right.intrinsic.msgVersion = frame_Version;
            inputFramePair->right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            inputFramePair->right.extrinsic.hasFixedTransform = true;

            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00173265;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0115965;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322039;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946654;

            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.2624;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.0862;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27392;

            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.301027;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = -0.142888;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66636;

            inputFramePair->right.data.msgVersion = array3D_Version;
            inputFramePair->right.data.rows = static_cast<asn1SccT_UInt32>(inputImageRight.rows);
            inputFramePair->right.data.cols = static_cast<asn1SccT_UInt32>(inputImageRight.cols);
            inputFramePair->right.data.channels = static_cast<asn1SccT_UInt32>(inputImageRight.channels());
            inputFramePair->right.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageRight.depth());
            inputFramePair->right.data.rowSize = inputImageRight.step[0];
            inputFramePair->right.data.data.nCount = static_cast<int>(inputFramePair->right.data.rows * inputFramePair->right.data.rowSize);
            memcpy(inputFramePair->right.data.data.arr, inputImageRight.data, static_cast<size_t>(inputFramePair->right.data.data.nCount));
        }

        // Loads a disparity image
        cv::Mat cvDispImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieDisparityDeg3FilteredEdres.exr", cv::IMREAD_ANYDEPTH);

        // Convert it to a frame
        asn1SccFrame_Initialize(dispImage);

        {
            dispImage->msgVersion = frame_Version;
            dispImage->metadata.msgVersion = frame_Version;
            dispImage->metadata.status = asn1Sccstatus_VALID;
            dispImage->metadata.mode = asn1Sccmode_GRAY;

            dispImage->metadata.pixelModel = asn1Sccpix_DISP;
            dispImage->metadata.pixelCoeffs.arr[0] = 1.;
            dispImage->metadata.pixelCoeffs.arr[1] = 0.;
            dispImage->metadata.pixelCoeffs.arr[2] = 0.270282;
            dispImage->metadata.pixelCoeffs.arr[3] = 3;
            dispImage->metadata.pixelCoeffs.arr[4] = 184;

            dispImage->intrinsic.msgVersion = frame_Version;
            dispImage->intrinsic.cameraModel = asn1Scccam_PINHOLE;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            dispImage->extrinsic.hasFixedTransform = true;

            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00173265;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0115965;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322039;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946654;

            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.2624;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.0862;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27392;

            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.295922;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = 0.124542;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66978;

            dispImage->data.msgVersion = array3D_Version;
            dispImage->data.rows = static_cast<asn1SccT_UInt32>(cvDispImage.rows);
            dispImage->data.cols = static_cast<asn1SccT_UInt32>(cvDispImage.cols);
            dispImage->data.channels = static_cast<asn1SccT_UInt32>(cvDispImage.channels());
            dispImage->data.depth = static_cast<asn1SccArray3D_depth_t>(cvDispImage.depth());
            dispImage->data.rowSize = cvDispImage.step[0];
            dispImage->data.data.nCount = static_cast<int>(dispImage->data.rows * dispImage->data.rowSize);
            memcpy(dispImage->data.data.arr, cvDispImage.data, static_cast<size_t>(dispImage->data.data.nCount));
        }

        // Send input data to DFN
        vme->framePairInput(*inputFramePair);
        vme->disparityInput(*dispImage);

        // Run DFN
        vme->process();
    }

    // Second Image
    {
        // Prepare input data
        cv::Mat inputImageLeftNext = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectDeg3LeftNext.png", cv::IMREAD_GRAYSCALE);
        cv::Mat inputImageRightNext = cv::imread("../tests/Data/Images/MinnieStereo/MinnieRectDeg3RightNext.png", cv::IMREAD_GRAYSCALE);

        // Initialise a frame and set its metadata
        asn1SccFramePair_Initialize(inputFramePair);

        // Fill the metadata
        inputFramePair->msgVersion = frame_Version;
        inputFramePair->baseline = -0.270268442641143;

        // Left frame
        {
            inputFramePair->left.msgVersion = frame_Version;
            inputFramePair->left.metadata.msgVersion = frame_Version;
            inputFramePair->left.metadata.status = asn1Sccstatus_VALID;
            inputFramePair->left.metadata.pixelModel = asn1Sccpix_UNDEF;
            inputFramePair->left.metadata.mode = asn1Sccmode_GRAY;

            inputFramePair->left.intrinsic.msgVersion = frame_Version;
            inputFramePair->left.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            inputFramePair->left.intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            inputFramePair->left.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00142314;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0098489;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322307;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946583;

            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.1466;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.1867;
            inputFramePair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27527;

            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.295922;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = 0.124542;
            inputFramePair->left.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66978;

            inputFramePair->left.data.msgVersion = array3D_Version;
            inputFramePair->left.data.rows = static_cast<asn1SccT_UInt32>(inputImageLeftNext.rows);
            inputFramePair->left.data.cols = static_cast<asn1SccT_UInt32>(inputImageLeftNext.cols);
            inputFramePair->left.data.channels = static_cast<asn1SccT_UInt32>(inputImageLeftNext.channels());
            inputFramePair->left.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageLeftNext.depth());
            inputFramePair->left.data.rowSize = inputImageLeftNext.step[0];
            inputFramePair->left.data.data.nCount = static_cast<int>(inputFramePair->left.data.rows * inputFramePair->left.data.rowSize);
            memcpy(inputFramePair->left.data.data.arr, inputImageLeftNext.data, static_cast<size_t>(inputFramePair->left.data.data.nCount));
        }

        // Right frame
        {
            inputFramePair->right.msgVersion = frame_Version;
            inputFramePair->right.metadata.msgVersion = frame_Version;
            inputFramePair->right.metadata.status = asn1Sccstatus_VALID;
            inputFramePair->right.metadata.pixelModel = asn1Sccpix_UNDEF;
            inputFramePair->right.metadata.mode = asn1Sccmode_GRAY;

            inputFramePair->right.intrinsic.msgVersion = frame_Version;
            inputFramePair->right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            inputFramePair->right.intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            inputFramePair->right.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            inputFramePair->right.extrinsic.hasFixedTransform = true;

            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00142314;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0098489;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322307;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946583;

            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.1466;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.1867;
            inputFramePair->right.extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27527;

            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.301027;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = -0.142888;
            inputFramePair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66636;

            inputFramePair->right.data.msgVersion = array3D_Version;
            inputFramePair->right.data.rows = static_cast<asn1SccT_UInt32>(inputImageRightNext.rows);
            inputFramePair->right.data.cols = static_cast<asn1SccT_UInt32>(inputImageRightNext.cols);
            inputFramePair->right.data.channels = static_cast<asn1SccT_UInt32>(inputImageRightNext.channels());
            inputFramePair->right.data.depth = static_cast<asn1SccArray3D_depth_t>(inputImageRightNext.depth());
            inputFramePair->right.data.rowSize = inputImageRightNext.step[0];
            inputFramePair->right.data.data.nCount = static_cast<int>(inputFramePair->right.data.rows * inputFramePair->right.data.rowSize);
            memcpy(inputFramePair->right.data.data.arr, inputImageRightNext.data, static_cast<size_t>(inputFramePair->right.data.data.nCount));
        }

        // Loads a disparity image
        cv::Mat cvdispImage = cv::imread("../tests/Data/Images/MinnieStereo/MinnieDisparityDeg3FilteredEdresNext.exr", cv::IMREAD_ANYDEPTH);

        // Convert it to a frame
        asn1SccFrame_Initialize(dispImage);
        {
            dispImage->msgVersion = frame_Version;
            dispImage->metadata.msgVersion = frame_Version;
            dispImage->metadata.status = asn1Sccstatus_VALID;
            dispImage->metadata.mode = asn1Sccmode_GRAY;

            dispImage->metadata.pixelModel = asn1Sccpix_DISP;
            dispImage->metadata.pixelCoeffs.arr[0] = 1.;
            dispImage->metadata.pixelCoeffs.arr[1] = 0.;
            dispImage->metadata.pixelCoeffs.arr[2] = 0.270282;
            dispImage->metadata.pixelCoeffs.arr[3] = 3;
            dispImage->metadata.pixelCoeffs.arr[4] = 184;

            dispImage->intrinsic.msgVersion = frame_Version;
            dispImage->intrinsic.cameraModel = asn1Scccam_PINHOLE;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[0] = 339.532;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[1] = 0;
            dispImage->intrinsic.cameraMatrix.arr[0].arr[2] = 318.834;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[0] = 0;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[1] = 339.532;
            dispImage->intrinsic.cameraMatrix.arr[1].arr[2] = 205.771;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[0] = 0;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[1] = 0;
            dispImage->intrinsic.cameraMatrix.arr[2].arr[2] = 1;

            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[0] = -0.00142314;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[1] = -0.0098489;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[2] = -0.322307;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.orientation.arr[3] = 0.946583;

            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[0] = -75.1466;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[1] = -11.1867;
            dispImage->extrinsic.pose_fixedFrame_robotFrame.data.translation.arr[2] = -4.27527;

            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0] = -0.631666;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1] = 0.623851;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2] = -0.318227;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3] = 0.332476;

            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[0] = 0.295922;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[1] = 0.124542;
            dispImage->extrinsic.pose_robotFrame_sensorFrame.data.translation.arr[2] = 1.66978;

            dispImage->data.msgVersion = array3D_Version;
            dispImage->data.rows = static_cast<asn1SccT_UInt32>(cvdispImage.rows);
            dispImage->data.cols = static_cast<asn1SccT_UInt32>(cvdispImage.cols);
            dispImage->data.channels = static_cast<asn1SccT_UInt32>(cvdispImage.channels());
            dispImage->data.depth = static_cast<asn1SccArray3D_depth_t>(cvdispImage.depth());
            dispImage->data.rowSize = cvdispImage.step[0];
            dispImage->data.data.nCount = static_cast<int>(dispImage->data.rows * dispImage->data.rowSize);
            memcpy(dispImage->data.data.arr, cvdispImage.data, static_cast<size_t>(dispImage->data.data.nCount));
        }

        // Send input data to DFN
        vme->framePairInput(*inputFramePair);
        vme->disparityInput(*dispImage);

        // Run DFN
        vme->process();
    }

    // Query output data from DFN
    const asn1SccTransformWithCovariance& output = vme->poseOutput();

    REQUIRE( output.metadata.msgVersion == transformWithCovariance_version );
    REQUIRE( output.data.orientation.arr[0]*output.data.orientation.arr[0] +
             output.data.orientation.arr[1]*output.data.orientation.arr[1] +
             output.data.orientation.arr[2]*output.data.orientation.arr[2] +
             output.data.orientation.arr[3]*output.data.orientation.arr[3] == Approx(1.).epsilon(1e-6) );
    REQUIRE( output.data.translation.arr[0] != 0 );
    REQUIRE( output.data.translation.arr[1] != 0 );
    REQUIRE( output.data.translation.arr[2] != 0 );

    // Cleanup
    delete(vme);
    delete(inputFramePair);
    delete(dispImage);
}


/** @} */
