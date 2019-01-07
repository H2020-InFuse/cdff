/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoDegradation.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Test application for the DFN StereoDegradation.
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

#include <StereoDegradation/StereoDegradation.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class StereoDegradationTestInterface : public DFNTestInterface
{
public:
    StereoDegradationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~StereoDegradationTestInterface();

private:
    CDFF::DFN::StereoDegradation::StereoDegradation degradation;

    cv::Mat inputImageLeft;
    cv::Mat inputImageRight;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

StereoDegradationTestInterface::StereoDegradationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    degradation()
{
    SetDFN(&degradation);

    inputImageLeft = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRawLeft.png", cv::IMREAD_GRAYSCALE);
    inputImageRight = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRawRight.png", cv::IMREAD_GRAYSCALE);

    // Initialise a frame pair and set its metadata
    asn1SccFramePair *inputFramePair = new asn1SccFramePair();
    asn1SccFramePair_Initialize(inputFramePair);

    // Fill the metadata
    inputFramePair->msgVersion = frame_Version;

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

    degradation.originalImagePairInput(*inputFramePair);
    outputWindowName = "Degraded Image Pair";

    delete(inputFramePair);
}

StereoDegradationTestInterface::~StereoDegradationTestInterface()
{
}

void StereoDegradationTestInterface::SetupParameters()
{
    AddParameter("StereoDegradationParams", "xratio", 2, 25, 1);
    AddParameter("StereoDegradationParams", "yratio", 2, 25, 1);
    AddParameter("StereoDegradationParams", "method", 3, 5, 1);
}

void StereoDegradationTestInterface::DisplayResult()
{
    // Fetch the resulting degraded image
    asn1SccFramePair* res =  new asn1SccFramePair();
    *res = degradation.degradedImagePairOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the degraded image as a cv::Mat for display
    cv::Mat degradedLeft = cv::Mat(static_cast<int>(res->left.data.rows), static_cast<int>(res->left.data.cols),
                                   CV_MAKETYPE(static_cast<int>(res->left.data.depth), static_cast<int>(res->left.data.channels)),
                                   res->left.data.data.arr, res->left.data.rowSize);

    cv::Mat degradedRight = cv::Mat(static_cast<int>(res->right.data.rows), static_cast<int>(res->right.data.cols),
                                    CV_MAKETYPE(static_cast<int>(res->right.data.depth), static_cast<int>(res->right.data.channels)),
                                    res->right.data.data.arr, res->right.data.rowSize);

    cv::Mat degraded;
    cv::hconcat(degradedLeft, degradedRight, degraded);

    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, degraded);
    cv::resizeWindow(outputWindowName, degraded.cols, degraded.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    StereoDegradationTestInterface* interface = new StereoDegradationTestInterface("StereoDegradation", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
