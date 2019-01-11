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
 * Test application for the DFN ColorConversion.
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

#include <ColorConversion/ColorConversion.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class ColorConversionTestInterface : public DFNTestInterface
{
public:
    ColorConversionTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~ColorConversionTestInterface();

private:
    CDFF::DFN::ColorConversion::ColorConversion conversion;

    cv::Mat inputImage;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

ColorConversionTestInterface::ColorConversionTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    conversion()
{
    SetDFN(&conversion);

    inputImage = cv::imread("../../tests/Data/Images/LabChairLeft.png", cv::IMREAD_COLOR);

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

    conversion.originalImageInput(*inputFrame);
    outputWindowName = "Converted Image";

    delete(inputFrame);
}

ColorConversionTestInterface::~ColorConversionTestInterface()
{
}

void ColorConversionTestInterface::SetupParameters()
{
    AddParameter("ColorConversionParams", "targetMode", 1, 13, 1);
}

void ColorConversionTestInterface::DisplayResult()
{
    // Fetch the resulting converted image
    asn1SccFrame* res =  new asn1SccFrame();
    *res = conversion.convertedImageOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the converted image as a cv::Mat for display
    cv::Mat converted = cv::Mat(static_cast<int>(res->data.rows), static_cast<int>(res->data.cols),
                               CV_MAKETYPE(static_cast<int>(res->data.depth), static_cast<int>(res->data.channels)),
                               res->data.data.arr, res->data.rowSize);


    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, converted);
    cv::resizeWindow(outputWindowName, converted.cols, converted.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    ColorConversionTestInterface* interface = new ColorConversionTestInterface("ColorConversion", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
