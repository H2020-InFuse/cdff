/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageRectificationEdres.cpp
 * @date 28/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Test application for the DFN ImageRectificationEdres.
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

#include <ImageRectification/ImageRectificationEdres.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>

class ImageRectificationEdresTestInterface : public DFNTestInterface
{
public:
    ImageRectificationEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~ImageRectificationEdresTestInterface();

private:
    CDFF::DFN::ImageRectification::ImageRectificationEdres rectification;

    cv::Mat inputImage;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

ImageRectificationEdresTestInterface::ImageRectificationEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    rectification()
{
    SetDFN(&rectification);

    inputImage = cv::imread("../../tests/Data/Images/EdresImages/NavL.png", cv::IMREAD_GRAYSCALE);

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

    rectification.originalImageInput(*inputFrame);
    outputWindowName = "Rectified Image";

    delete(inputFrame);
}

ImageRectificationEdresTestInterface::~ImageRectificationEdresTestInterface()
{
}

void ImageRectificationEdresTestInterface::SetupParameters()
{
    AddParameter("ImageRectificationEdresParams", "xratio", 2, 25, 1);
    AddParameter("ImageRectificationEdresParams", "yratio", 2, 25, 1);
    AddSignedParameter("ImageRectificationEdresParams", "xshift", 0, 1000, 1);
    AddSignedParameter("ImageRectificationEdresParams", "yshift", 0, 1000, 1);
    AddParameter("ImageRectificationEdresParams", "outType", 5, 8, 1);
    AddStringParameter("ImageRectificationEdresParams", "mapFile", "../../tests/Data/Images/EdresImages/camL");
}

void ImageRectificationEdresTestInterface::DisplayResult()
{
    // Fetch the resulting rectified image
    asn1SccFrame* res =  new asn1SccFrame();
    *res = rectification.rectifiedImageOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the rectified image as a cv::Mat for display
    cv::Mat rectified = cv::Mat(static_cast<int>(res->data.rows), static_cast<int>(res->data.cols),
                               CV_MAKETYPE(static_cast<int>(res->data.depth), static_cast<int>(res->data.channels)),
                               res->data.data.arr, res->data.rowSize);


    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, rectified);
    cv::resizeWindow(outputWindowName, rectified.cols, rectified.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    ImageRectificationEdresTestInterface* interface = new ImageRectificationEdresTestInterface("ImageRectificationEdres", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
