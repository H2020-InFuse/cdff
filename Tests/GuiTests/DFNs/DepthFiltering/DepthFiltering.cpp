/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DepthFiltering.cpp
 * @date 4/9/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN DepthFiltering.
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

#include <DepthFiltering/ConvolutionFilter.hpp>

#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class DepthFilteringTestInterface : public DFNTestInterface
{
public:
    DepthFilteringTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
    ~DepthFilteringTestInterface();

private:
    CDFF::DFN::DepthFiltering::ConvolutionFilter *filter;
    cv::Mat cvImage;
    FrameWrapper::FrameConstPtr inputImage;
    std::string outputWindowName;

    void SetupParameters();
    void DisplayResult();
};

DepthFilteringTestInterface::DepthFilteringTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
        : DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
{
    filter = new CDFF::DFN::DepthFiltering::ConvolutionFilter();
    SetDFN(filter);

    cv::Mat image = cv::imread("../../tests/Data/Images/depth_img.jpg", cv::IMREAD_GRAYSCALE);
    inputImage = Converters::MatToFrameConverter().Convert(image);

    filter->frameInput(*inputImage);
    outputWindowName = "Depth Filtering Result";
}

DepthFilteringTestInterface::~DepthFilteringTestInterface()
{
    delete(inputImage);
    delete(filter);
}

void DepthFilteringTestInterface::SetupParameters()
{
    AddParameter("GeneralParameters", "KernelSize", 5, 20, 1);
}

void DepthFilteringTestInterface::DisplayResult()
{
    const FrameWrapper::Frame& filtered_image = filter->frameOutput();
    cv::Mat filtered_cv_image = Converters::FrameToMatConverter().Convert(&filtered_image);

    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, filtered_cv_image);
    PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
    PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
}

int main(int argc, char** argv)
{
    DepthFilteringTestInterface interface("DepthFiltering", 100, 40);
    interface.Run();
};

/** @} */
