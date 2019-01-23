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
 * Test application for the DFN StereoRectificationEdres.
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

#include <StereoRectification/StereoRectificationEdres.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class StereoRectificationEdresTestInterface : public DFNTestInterface
{
public:
    StereoRectificationEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~StereoRectificationEdresTestInterface();

private:
    CDFF::DFN::StereoRectification::StereoRectificationEdres rectification;

    cv::Mat inputImageLeft;
    cv::Mat inputImageRight;

    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

StereoRectificationEdresTestInterface::StereoRectificationEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    rectification()
{
    SetDFN(&rectification);

    inputImageLeft = cv::imread("../../tests/Data/Images/EdresImages/NavL.png", cv::IMREAD_GRAYSCALE);
    inputImageRight = cv::imread("../../tests/Data/Images/EdresImages/NavR.png", cv::IMREAD_GRAYSCALE);

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

    rectification.originalStereoPairInput(*inputFramePair);
    outputWindowName = "rectified Image";

    delete(inputFramePair);
}

StereoRectificationEdresTestInterface::~StereoRectificationEdresTestInterface()
{
}

void StereoRectificationEdresTestInterface::SetupParameters()
{
    AddParameter("StereoRectificationEdresParams", "xratio", 2, 25, 1);
    AddParameter("StereoRectificationEdresParams", "yratio", 2, 25, 1);
    AddParameter("StereoRectificationEdresParams", "outType", 5, 8, 1);
    AddStringParameter("StereoRectificationEdresParams", "mapFileLeft", "../../tests/Data/Images/EdresImages/camL");
    AddStringParameter("StereoRectificationEdresParams", "mapFileRight", "../../tests/Data/Images/EdresImages/camR");
}

void StereoRectificationEdresTestInterface::DisplayResult()
{
    // Fetch the resulting degraded image
    asn1SccFramePair* res =  new asn1SccFramePair();
    *res = rectification.rectifiedStereoPairOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the rectified image as a cv::Mat for display
    cv::Mat rectifiedLeft = cv::Mat(static_cast<int>(res->left.data.rows), static_cast<int>(res->left.data.cols),
                                   CV_MAKETYPE(static_cast<int>(res->left.data.depth), static_cast<int>(res->left.data.channels)),
                                   res->left.data.data.arr, res->left.data.rowSize);

    cv::Mat rectifiedRight = cv::Mat(static_cast<int>(res->right.data.rows), static_cast<int>(res->right.data.cols),
                                    CV_MAKETYPE(static_cast<int>(res->right.data.depth), static_cast<int>(res->right.data.channels)),
                                    res->right.data.data.arr, res->right.data.rowSize);

    cv::Mat rectified;
    cv::hconcat(rectifiedLeft, rectifiedRight, rectified);

    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, rectified);
    cv::resizeWindow(outputWindowName, rectified.cols, rectified.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    StereoRectificationEdresTestInterface* interface = new StereoRectificationEdresTestInterface("StereoRectificationEdres", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
