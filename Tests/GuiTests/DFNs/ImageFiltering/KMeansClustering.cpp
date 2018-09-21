#include <memory>

#include <Errors/Assert.hpp>
#include <Types/CPP/Frame.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>

#include <ImageFiltering/KMeansClustering.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <opencv2/core.hpp>
#include <iostream>

using namespace CDFF::DFN;
using namespace FrameWrapper;
using namespace Array3DWrapper;
using namespace BaseTypesWrapper;

class KMeansClusteringTest: public DFNTestInterface
{
public:
    KMeansClusteringTest();
    ~KMeansClusteringTest() override = default;

private:

    void SetupParameters() override;
    void DisplayResult() override;

    // State Variables
    cv::Mat srcImage;
    std::unique_ptr<KMeansClustering> kmeans_impl;
    const std::string outputWindowName = "K-Means Clustering";
};


KMeansClusteringTest::KMeansClusteringTest()
        : DFNTestInterface("K-Means Clustering", 100, 40) {
    kmeans_impl.reset(new KMeansClustering());
    SetDFN(kmeans_impl.get());

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

    const size_t datasize = srcImage.total() * srcImage.elemSize();
    assert(sizeof(inFrame->data.data.arr) > datasize);
    std::copy(srcImage.data, srcImage.data + datasize, inFrame->data.data.arr);

    inFrame->data.data.nCount = static_cast<int>(srcImage.total() * srcImage.elemSize());
    kmeans_impl->imageInput(*inFrame);
}

void KMeansClusteringTest::SetupParameters()
{
    AddParameter("KMeansClustering", "num_centers",
            KMeansClustering::DefaultParameters.num_centers, 48);
    AddParameter("KMeansClustering", "max_iterations",
                 KMeansClustering::DefaultParameters.max_iterations, 100);
    AddParameter("KMeansClustering", "tolerance",
            KMeansClustering::DefaultParameters.tolerance, 10, .1);
}

void KMeansClusteringTest::DisplayResult()
{
    const FrameWrapper::Frame& kmeans_image = kmeans_impl->imageOutput();
    const cv::Mat clusters = Converters::FrameToMatConverter().Convert(&kmeans_image);

    ASSERT(srcImage.size() == clusters.size(),
           "Raw and segmented frames should have the same size");

    cv::Mat srcImage_rescaled;
    cv::normalize(srcImage, srcImage_rescaled, 0.0, 1.0, cv::NORM_MINMAX);

    cv::Mat srcImage_tmp, srcImage_rgb;
    srcImage_rescaled.convertTo(srcImage_tmp, CV_8UC3, 255);
    cv::applyColorMap(srcImage_tmp, srcImage_rgb, cv::COLORMAP_PARULA);

    cv::Mat clusters_rescaled;
    cv::normalize(clusters, clusters_rescaled, 255, 0, cv::NORM_MINMAX);

    cv::Mat clusters_cm;
    cv::applyColorMap(clusters_rescaled, clusters_cm, cv::COLORMAP_JET);

    cv::Size canvas_size{
            10 + srcImage.cols + 10 + clusters.cols + 10,
            10 + srcImage.rows + 10
    };
    cv::Mat canvas(canvas_size, CV_8UC3, cv::Scalar(215, 217, 220));

    cv::Rect srcImage_roi{ 10, 10, srcImage.cols, srcImage.rows };
    srcImage_rgb.copyTo( canvas(srcImage_roi) );

    cv::Rect clusters_cm_roi{
            10 + srcImage.cols + 10, 10, clusters.cols, clusters.rows
    };
    clusters_cm.copyTo( canvas(clusters_cm_roi) );

    cv::namedWindow(outputWindowName, cv::WINDOW_NORMAL);
    cv::imshow(outputWindowName, canvas);
}

int main(int argc, char** argv)
{
    KMeansClusteringTest testImpl;
    testImpl.Run();
}
