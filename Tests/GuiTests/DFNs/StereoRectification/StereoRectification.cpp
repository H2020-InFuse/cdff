/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoRectification.cpp
 * @date 26/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Test application for the DFN StereoRectification.
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

#include <StereoRectification/StereoRectification.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class StereoRectificationTestInterface : public DFNTestInterface
{
public:
    StereoRectificationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~StereoRectificationTestInterface();

private:
    CDFF::DFN::StereoRectification::StereoRectification rectification;

    cv::Mat inputImageLeft;
    cv::Mat inputImageRight;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

StereoRectificationTestInterface::StereoRectificationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    rectification()
{
    SetDFN(&rectification);

    inputImageLeft = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRawLeft.png", cv::IMREAD_GRAYSCALE);
    inputImageRight = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRawRight.png", cv::IMREAD_GRAYSCALE);

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

        std::string sensorId = "tisc-33UP2000_17810171";
        inputFrame.intrinsic.sensorId.nCount = static_cast<int>(sensorId.size() +1);
        memcpy(inputFrame.intrinsic.sensorId.arr, sensorId.data(), static_cast<size_t>(inputFrame.intrinsic.sensorId.nCount));

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

        std::string sensorId = "tisc-33UP2000_17810152";
        inputFrame.intrinsic.sensorId.nCount = static_cast<int>(sensorId.size() +1);
        memcpy(inputFrame.intrinsic.sensorId.arr, sensorId.data(), static_cast<size_t>(inputFrame.intrinsic.sensorId.nCount));

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

StereoRectificationTestInterface::~StereoRectificationTestInterface()
{
}

void StereoRectificationTestInterface::SetupParameters()
{
    AddParameter("StereoRectificationParams", "xratio", 2, 25, 1);
    AddParameter("StereoRectificationParams", "yratio", 2, 25, 1);
    AddParameter("StereoRectificationParams", "scaling", 0, 1, 0.1);
    AddParameter("StereoRectificationParams", "fisheye", 0, 1);
    AddStringParameter("StereoRectificationParams", "calibrationFilePath", "../../tests/Data/Images/MinnieStereo");
}

void StereoRectificationTestInterface::DisplayResult()
{
    // Fetch the resulting rectified image
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
    StereoRectificationTestInterface* interface = new StereoRectificationTestInterface("StereoRectification", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
