//
// Created by tch on 17.07.18.
//

#include <memory>

#include <opencv2/core.hpp>

#include <Errors/Assert.hpp>
#include <Types/CPP/Frame.hpp>
#include <ImageFiltering/NormalVectorExtraction.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <Types/C/Frame.h>
#include <iostream>

using namespace dfn_ci;
using namespace FrameWrapper;

class NormalVectorExtractionTest: public DFNTestInterface
{
public:
    NormalVectorExtractionTest();

private:
    void DisplayResult() override;

    // State Variables
    cv::Mat srcImage;
    std::unique_ptr<NormalVectorExtraction> normal_extraction_impl;
    const std::string outputWindowName = "Normal Vector Extraction";
};

NormalVectorExtractionTest::NormalVectorExtractionTest()
    : DFNTestInterface("Normal Vector Extraction", 100, 40)
{
    normal_extraction_impl.reset(new NormalVectorExtraction());
    SetDFN(normal_extraction_impl.get());

    cv::FileStorage file_reader("../../tests/Data/depthImg_Mana.yml.gz", cv::FileStorage::READ);
    ASSERT(file_reader.isOpened(), "Unable to input image file");

    file_reader.getFirstTopLevelNode() >> srcImage;
    ASSERT(!srcImage.empty(), "Something went wrong when reading the image");

    FrameSharedPtr inFrame = NewSharedFrame();
    Initialize(*inFrame);
    SetFrameSize(*inFrame, srcImage.cols, srcImage.rows);
    SetDataDepth(*inFrame, sizeof(float) * 8);
    SetPixelSize(*inFrame, sizeof(float) * 3);
    SetRowSize(*inFrame, GetFrameWidth(*inFrame) * GetPixelSize(*inFrame));
    SetFrameMode(*inFrame, FrameWrapper::MODE_UNDEFINED);
    SetFrameStatus(*inFrame, FrameWrapper::STATUS_VALID);

    std::copy(srcImage.data, srcImage.data + (srcImage.total() * srcImage.elemSize()), inFrame->image.arr);
    inFrame->image.nCount = static_cast<int>(srcImage.total() * srcImage.elemSize());
    normal_extraction_impl->imageInput(*inFrame);
}

void NormalVectorExtractionTest::DisplayResult()
{
    const FrameWrapper::Frame& normals_frame = normal_extraction_impl->imageOutput();

    cv::Size frame_size(
        static_cast<int>(normals_frame.datasize.width),
        static_cast<int>(normals_frame.datasize.height));
    cv::Mat normals(frame_size, CV_32FC3, (void*) normals_frame.image.arr, normals_frame.row_size);

    ASSERT(srcImage.size() == normals.size(),
        "Raw and segmented frames should have the same size");

    // Rescale the data for visualization because OpenCV expects the coordinates
    // to be in [0; 1] range but ours are in the [-1; 1] range.
    normals = normals / 2.f + .5f;
    cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
    cv::imshow(outputWindowName, normals);
}

int main(int argc, char** argv)
{
    NormalVectorExtractionTest testImpl;
    testImpl.Run();
}