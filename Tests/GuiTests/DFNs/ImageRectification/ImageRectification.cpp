/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageRectification.cpp
 * @date 28/12/2018
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Test application for the DFN ImageRectification.
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

#include <ImageRectification/ImageRectification.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class ImageRectificationTestInterface : public DFNTestInterface
{
public:
    ImageRectificationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~ImageRectificationTestInterface();

private:
    CDFF::DFN::ImageRectification::ImageRectification rectification;

    cv::Mat inputImage;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

ImageRectificationTestInterface::ImageRectificationTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    rectification()
{
    SetDFN(&rectification);

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

    rectification.originalImageInput(*inputFrame);
    outputWindowName = "Rectified Image";

    delete(inputFrame);
}

ImageRectificationTestInterface::~ImageRectificationTestInterface()
{
}

void ImageRectificationTestInterface::SetupParameters()
{
    AddParameter("ImageRectificationParams", "xratio", 1, 25, 1);
    AddParameter("ImageRectificationParams", "yratio", 1, 25, 1);
    AddParameter("ImageRectificationParams", "scaling", 0, 1, 0.1);
    AddParameter("ImageRectificationParams", "centerPrincipalPoint", 0, 1);
    AddParameter("ImageRectificationParams", "fisheye", 0, 1);
}

void ImageRectificationTestInterface::DisplayResult()
{
    // Fetch the resulting degraded image
    asn1SccFrame* res =  new asn1SccFrame();
    *res = rectification.rectifiedImageOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the degraded image as a cv::Mat for display
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
    ImageRectificationTestInterface* interface = new ImageRectificationTestInterface("ImageRectification", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
