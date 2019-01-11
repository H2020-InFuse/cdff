/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityFilteringEdres.cpp
 * @date 07/01/2019
 * @author Raphaël Viards
 */

/*!
 * @addtogroup DFNsTest
 *
 * Test application for the DFN DisparityFilteringEdres.
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

#include <DisparityFiltering/DisparityFilteringEdres.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class DisparityFilteringEdresTestInterface : public DFNTestInterface
{
public:
    DisparityFilteringEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
    ~DisparityFilteringEdresTestInterface();

private:
    CDFF::DFN::DisparityFiltering::DisparityFilteringEdres filter;

    cv::Mat inputImage;
    std::string outputWindowName;

    void SetupParameters() override;
    void DisplayResult() override;
};

DisparityFilteringEdresTestInterface::DisparityFilteringEdresTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
    DFNTestInterface(dfnName, buttonWidth, buttonHeight),
    filter()
{
    SetDFN(&filter);

    inputImage = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieDisparityDeg3Edres.exr", cv::IMREAD_ANYDEPTH);

    // Initialise a frame and set its metadata
    asn1SccFrame *inputFrame = new asn1SccFrame();
    asn1SccFrame_Initialize(inputFrame);

    // Fill the metadata
    inputFrame->msgVersion = frame_Version;

    inputFrame->metadata.msgVersion = frame_Version;
    inputFrame->metadata.status = asn1Sccstatus_VALID;

    inputFrame->metadata.pixelModel = asn1Sccpix_DISP;
    inputFrame->metadata.pixelCoeffs.arr[0] = 1.;
    inputFrame->metadata.pixelCoeffs.arr[1] = 0.;
    inputFrame->metadata.pixelCoeffs.arr[2] = 0.270282;
    inputFrame->metadata.pixelCoeffs.arr[3] = 3;
    inputFrame->metadata.pixelCoeffs.arr[4] = 184;

    inputFrame->metadata.mode = asn1Sccmode_UNDEF;

    inputFrame->data.msgVersion = array3D_Version;
    inputFrame->data.rows = static_cast<asn1SccT_UInt32>(inputImage.rows);
    inputFrame->data.cols = static_cast<asn1SccT_UInt32>(inputImage.cols);
    inputFrame->data.channels = static_cast<asn1SccT_UInt32>(inputImage.channels());
    inputFrame->data.depth = static_cast<asn1SccArray3D_depth_t>(inputImage.depth());
    inputFrame->data.rowSize = inputImage.step[0];
    inputFrame->data.data.nCount = static_cast<int>(inputFrame->data.rows * inputFrame->data.rowSize);
    memcpy(inputFrame->data.data.arr, inputImage.data, static_cast<size_t>(inputFrame->data.data.nCount));

    filter.rawDisparityInput(*inputFrame);
    outputWindowName = "Filtered Disparity";

    delete(inputFrame);
}

DisparityFilteringEdresTestInterface::~DisparityFilteringEdresTestInterface()
{
}

void DisparityFilteringEdresTestInterface::SetupParameters()
{
    AddParameter("DisparityFilteringEdresParams", "trimWidth", 3, 100, 1);
    AddParameter("DisparityFilteringEdresParams", "connexityThresh", 2, 10, 0.1);
    AddParameter("DisparityFilteringEdresParams", "surfMin", 20, 1000, 10);
    AddParameter("DisparityFilteringEdresParams", "surfMax", 1000, 5000, 10);
}

void DisparityFilteringEdresTestInterface::DisplayResult()
{
    // Fetch the resulting filtered image
    asn1SccFrame* res =  new asn1SccFrame();
    *res = filter.filteredDisparityOutput();

    PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
    PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

    // Convert the filtered image as a cv::Mat for display
    cv::Mat filtered =  cv::Mat(static_cast<int>(res->data.rows), static_cast<int>(res->data.cols),
                                CV_MAKETYPE(static_cast<int>(res->data.depth), static_cast<int>(res->data.channels)),
                                res->data.data.arr, res->data.rowSize);

    // Apply a colormap
    cv::Mat filteredColor;
    double min,	max;
    cv::minMaxLoc(filtered, &min, &max);
    filtered.convertTo(filtered, CV_8U, 255 / (max - min), -255.0 * min / (max - min));
    cv::Mat mask = filtered > 0;
    cv::applyColorMap(filtered, filtered, 2);
    filtered.copyTo(filteredColor, mask);


    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, filteredColor);
    cv::resizeWindow(outputWindowName, filteredColor.cols, filteredColor.rows);
    cv::waitKey(500);

    delete(res);
}

int main(int argc, char** argv)
{
    DisparityFilteringEdresTestInterface* interface = new DisparityFilteringEdresTestInterface("DisparityFilteringEdres", 100, 40);
    interface->Run();
    delete(interface);
}

/** @} */
