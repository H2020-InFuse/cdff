/**
 * Unit tests for the DFN NormalVectorExtraction
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <ImageFiltering/NormalVectorExtraction.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace FrameWrapper;
using namespace Converters;
using namespace Array3DWrapper;
using namespace BaseTypesWrapper;

TEST_CASE( "Call to process (NormalVectorExtraction)", "[process]" )
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

    std::copy(input.data, input.data + (input.total() * input.elemSize()), inFrame->data.data.arr);
    inFrame->data.data.nCount = static_cast<int>(input.total() * input.elemSize());

	// Instantiate DFN
	std::unique_ptr<NormalVectorExtraction> normal_vector_extraction (new NormalVectorExtraction);

	// Send input data to DFN
	normal_vector_extraction->imageInput(*inFrame);

	// Run DFN
	normal_vector_extraction->process();

	// Query output data from DFN
	const Frame& output = normal_vector_extraction->imageOutput();

    // Check the DFN output image is not empty
    CHECK(output.data.rows == input.rows);
    CHECK(output.data.cols == input.cols);
}

/** @} */
