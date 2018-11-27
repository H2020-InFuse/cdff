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

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace BaseTypesWrapper;
using namespace Array3DWrapper;

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
    inFrame->metadata.mode = FrameMode::asn1Sccmode_XYZ;
    inFrame->metadata.status = FrameStatus::asn1Sccstatus_VALID;
    inFrame->data.rows = static_cast<T_UInt16>(srcImage.rows);
    inFrame->data.cols = static_cast<T_UInt16>(srcImage.cols);
    inFrame->data.rowSize = static_cast<T_UInt32>(srcImage.step[0]);
    inFrame->data.channels = 3;
    inFrame->data.depth = Array3DDepth::asn1Sccdepth_32S;


    std::copy(srcImage.data, srcImage.data + (srcImage.total() * srcImage.elemSize()), inFrame->data.data.arr);
    inFrame->data.data.nCount = static_cast<int>(srcImage.total() * srcImage.elemSize());
    normal_extraction_impl->imageInput(*inFrame);
}

void NormalVectorExtractionTest::DisplayResult()
{
    const FrameWrapper::Frame& normals_frame = normal_extraction_impl->imageOutput();

    cv::Size frame_size(
        static_cast<int>(normals_frame.data.cols),
        static_cast<int>(normals_frame.data.rows));
    cv::Mat normals(frame_size, CV_32FC3, (void*) normals_frame.data.data.arr, normals_frame.data.rowSize);

    ASSERT(srcImage.size() == normals.size(),
        "Raw and segmented frames should have the same size");

    // Rescale the data for visualization because OpenCV expects the coordinates
    // to be in [0; 1] range but ours are in the [-1; 1] range.
    normals = normals / 2.f + .5f;
    cv::namedWindow(outputWindowName, cv::WINDOW_NORMAL);
    cv::imshow(outputWindowName, normals);
}

int main(int argc, char** argv)
{
    NormalVectorExtractionTest testImpl;
    testImpl.Run();
}