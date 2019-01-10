/**
 * Unit tests for the DFN KMeansClustering
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <boost/assert.hpp>
#include <ImageFiltering/KMeansClustering.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace Converters;
using namespace Array3DWrapper;
using namespace BaseTypesWrapper;

TEST_CASE( "Call to process (KMeansClustering)", "[process]" )
{
	// Prepare input data
    cv::FileStorage file_reader("../tests/Data/depthImg_Mana.yml.gz", cv::FileStorage::READ);
    ASSERT(file_reader.isOpened(), "Unable to input image file");
    cv::Mat input;
    file_reader.getFirstTopLevelNode() >> input;
    ASSERT(!input.empty(), "Something went wrong when reading the image");

    FrameSharedPtr inFrame = NewSharedFrame();
    Initialize(*inFrame);
    inFrame->metadata.mode = FrameMode::asn1Sccmode_XYZ;
    inFrame->metadata.status = FrameStatus::asn1Sccstatus_VALID;
    inFrame->data.rows = static_cast<T_UInt16>(input.rows);
    inFrame->data.cols = static_cast<T_UInt16>(input.cols);
    inFrame->data.rowSize = static_cast<T_UInt32>(input.step[0]);
    inFrame->data.channels = 3;
    inFrame->data.depth = Array3DDepth::asn1Sccdepth_32S;

    const size_t datasize = input.total() * input.elemSize();
    assert(sizeof(inFrame->data.data.arr) > datasize);
    std::copy(input.data, input.data + datasize, inFrame->data.data.arr);

    inFrame->data.data.nCount = static_cast<int>(input.total() * input.elemSize());

	// Instantiate DFN
	std::unique_ptr<KMeansClustering> kmeans_clustering (new KMeansClustering);

	// Send input data to DFN
	kmeans_clustering->imageInput(*inFrame);

	// Run DFN
	kmeans_clustering->process();

	// Query output data from DFN
	const Frame& output = kmeans_clustering->imageOutput();

	// Check the DFN output image is not empty
	BOOST_ASSERT(output.data.data.nCount > 0);
}

TEST_CASE( "Call to configure (KMeansClustering) ", "[configure]" )
{
    // Prepare input data
    cv::FileStorage file_reader("../tests/Data/depthImg_Mana.yml.gz", cv::FileStorage::READ);
    ASSERT(file_reader.isOpened(), "Unable to input image file");
    cv::Mat input;
    file_reader.getFirstTopLevelNode() >> input;
    ASSERT(!input.empty(), "Something went wrong when reading the image");

    FrameSharedPtr inFrame = NewSharedFrame();
    Initialize(*inFrame);
    inFrame->metadata.mode = FrameMode::asn1Sccmode_XYZ;
    inFrame->metadata.status = FrameStatus::asn1Sccstatus_VALID;
    inFrame->data.rows = static_cast<T_UInt16>(input.rows);
    inFrame->data.cols = static_cast<T_UInt16>(input.cols);
    inFrame->data.rowSize = static_cast<T_UInt32>(input.step[0]);
    inFrame->data.channels = 3;
    inFrame->data.depth = Array3DDepth::asn1Sccdepth_32S;

    const size_t datasize = input.total() * input.elemSize();
    assert(sizeof(inFrame->data.data.arr) > datasize);
    std::copy(input.data, input.data + datasize, inFrame->data.data.arr);

    inFrame->data.data.nCount = static_cast<int>(input.total() * input.elemSize());

    // Instantiate DFN
    std::unique_ptr<KMeansClustering> kmeans_clustering (new KMeansClustering);

    // Setup DFN
    kmeans_clustering->setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/KMeansClustering.yaml");
    kmeans_clustering->configure();

    // Send input data to DFN
    kmeans_clustering->imageInput(*inFrame);

    // Run DFN
    kmeans_clustering->process();

    // Query output data from DFN
    const Frame& output = kmeans_clustering->imageOutput();

    // Check the DFN output image is not empty
    BOOST_ASSERT(output.data.data.nCount > 0);
}

/** @} */
