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
 * Test application for the DFN ImageDegradation.
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

#include <ImageDegradation/ImageDegradation.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class ImageDegradationTestInterface : public DFNTestInterface
{
public:
    ImageDegradationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~ImageDegradationTestInterface();

private:
    CDFF::DFN::ImageDegradation::ImageDegradation degradation;

    cv::Mat inputImage;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

ImageDegradationTestInterface::ImageDegradationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    degradation()
{
    SetDFN(&degradation);

    inputImage = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRawLeft.png", cv::IMREAD_GRAYSCALE);

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

    degradation.originalImageInput(*inputFrame);
    outputWindowName = "Degraded Image";

    delete(inputFrame);
}

ImageDegradationTestInterface::~ImageDegradationTestInterface()
{
}

void ImageDegradationTestInterface::SetupParameters()
{
    AddParameter("ImageDegradationParams", "xratio", 2, 25, 1);
    AddParameter("ImageDegradationParams", "yratio", 2, 25, 1);
    AddParameter("ImageDegradationParams", "method", 3, 5, 1);
}

void ImageDegradationTestInterface::DisplayResult()
{
    // Fetch the resulting degraded image
    asn1SccFrame* res =  new asn1SccFrame();
    *res = degradation.degradedImageOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the degraded image as a cv::Mat for display
    cv::Mat degraded = cv::Mat(static_cast<int>(res->data.rows), static_cast<int>(res->data.cols),
                               CV_MAKETYPE(static_cast<int>(res->data.depth), static_cast<int>(res->data.channels)),
                               res->data.data.arr, res->data.rowSize);


    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, degraded);
    cv::resizeWindow(outputWindowName, degraded.cols, degraded.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    ImageDegradationTestInterface* interface = new ImageDegradationTestInterface("ImageDegradation", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
