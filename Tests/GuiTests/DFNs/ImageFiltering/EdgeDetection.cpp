//
// Created by tch on 13.07.18.
//

#include <memory>

#include <Errors/Assert.hpp>
#include <Types/CPP/Frame.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <ImageFiltering/EdgeDetection.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;

class EdgeDetectionTest: public DFNTestInterface
{
public:
    EdgeDetectionTest();
    ~EdgeDetectionTest() override = default;

private:

    void SetupParameters() override;
    void DisplayResult() override;

    // State Variables
    cv::Mat srcImage;
    std::unique_ptr<EdgeDetection> edge_detection_impl;
    const std::string outputWindowName = "Edge Detection";
};


EdgeDetectionTest::EdgeDetectionTest()
    : DFNTestInterface("Edge Detection", 100, 40)
{
    edge_detection_impl.reset(new EdgeDetection());
    SetDFN(edge_detection_impl.get());

    srcImage = cv::imread("../../tests/Data/Images/LabChairLeft.png", cv::IMREAD_COLOR);
    ASSERT(!srcImage.empty(), "Unable to open source image.")

    FrameSharedConstPtr inputImage = Converters::MatToFrameConverter().ConvertShared(srcImage);
    edge_detection_impl->imageInput(*inputImage); 
}


void EdgeDetectionTest::SetupParameters()
{
    AddParameter("EdgeDetection", "NoiseReductionKernelSize",
        EdgeDetection::DefaultParameters.NoiseReductionKernelSize, 13);
    AddParameter("EdgeDetection", "CannyLowThreshold",
        EdgeDetection::DefaultParameters.CannyLowThreshold, 150.0, 1);
    AddParameter("EdgeDetection", "CannyHighThreshold",
        EdgeDetection::DefaultParameters.CannyHighThreshold, 450.0, 1);
}

void EdgeDetectionTest::DisplayResult()
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
    EdgeDetectionTest testImpl;
    testImpl.Run();
}
