//
// Created by tch on 13.07.18.
//

#include <memory>

#include <Errors/Assert.hpp>
#include <Types/CPP/Frame.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <ImageFiltering/CannyEdgeDetection.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;

class CannyEdgeDetectionTest: public DFNTestInterface
{
public:
    CannyEdgeDetectionTest();
    ~CannyEdgeDetectionTest() override = default;

private:

    void SetupParameters() override;
    void DisplayResult() override;

    // State Variables
    cv::Mat srcImage;
    std::unique_ptr<CannyEdgeDetection> edge_detection_impl;
    const std::string outputWindowName = "Edge Detection";
};


CannyEdgeDetectionTest::CannyEdgeDetectionTest()
    : DFNTestInterface("Edge Detection", 100, 40)
{
    edge_detection_impl.reset(new CannyEdgeDetection());
    SetDFN(edge_detection_impl.get());

    srcImage = cv::imread("../../tests/Data/Images/LabChairLeft.png", cv::IMREAD_COLOR);
    ASSERT(!srcImage.empty(), "Unable to open source image.")

    FrameSharedConstPtr inputImage = Converters::MatToFrameConverter().ConvertShared(srcImage);
    edge_detection_impl->imageInput(*inputImage); 
}


void CannyEdgeDetectionTest::SetupParameters()
{
    AddParameter("CannyEdgeDetection", "NoiseReductionKernelSize",
        CannyEdgeDetection::DefaultParameters.NoiseReductionKernelSize, 13);
    AddParameter("CannyEdgeDetection", "CannyLowThreshold",
        CannyEdgeDetection::DefaultParameters.CannyLowThreshold, 150.0, 1);
    AddParameter("CannyEdgeDetection", "CannyHighThreshold",
        CannyEdgeDetection::DefaultParameters.CannyHighThreshold, 450.0, 1);
}

void CannyEdgeDetectionTest::DisplayResult()
{
    const FrameWrapper::Frame& edge_image = edge_detection_impl->imageOutput();
    cv::Mat processed_frame = Converters::FrameToMatConverter().Convert(&edge_image);
    ASSERT(srcImage.size() == processed_frame.size(),
        "Raw and segmented frames should have the same size");

    cv::Mat segmented_frame_rgb;
    cv::cvtColor(processed_frame, segmented_frame_rgb, cv::COLOR_GRAY2BGR);

    cv::Size canvas_size{
        10 + srcImage.cols + 10 + processed_frame.cols + 10,
        10 + srcImage.rows + 10
    };
    cv::Mat canvas(canvas_size, CV_8UC3, cv::Scalar(215, 217, 220));

    cv::Rect srcImage_roi{ 10, 10, srcImage.cols, srcImage.rows };
    srcImage.copyTo( canvas(srcImage_roi) );

    cv::Rect processed_frame_roi{
        10 + srcImage.cols + 10, 10, processed_frame.cols, processed_frame.rows
    };
    segmented_frame_rgb.copyTo( canvas(processed_frame_roi) );

    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, canvas);
}

int main(int argc, char** argv)
{
    CannyEdgeDetectionTest testImpl;
    testImpl.Run();
}
