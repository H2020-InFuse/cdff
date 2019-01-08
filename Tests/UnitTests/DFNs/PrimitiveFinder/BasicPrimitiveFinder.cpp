/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Basic Primitive Finder.
 *
 *
 * @{
 */

#include <catch.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Converters/MatToFrameConverter.hpp>
#include <PrimitiveFinder/BasicPrimitiveFinder.hpp>


using namespace CDFF::DFN::PrimitiveFinder;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (BasicPrimitiveFinder)", "[process]" )
{
    // Prepare input data
    cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    FrameSharedConstPtr input = MatToFrameConverter().ConvertShared(gray);

    //Circle finder:
    std::string circle_string = "circle";
    asn1SccT_String circle;
    circle.nCount = circle_string.size();
    memcpy(circle.arr, circle_string.data(), circle_string.length());

    // Instantiate DFN
    std::unique_ptr<BasicPrimitiveFinder> primitive_finder (new BasicPrimitiveFinder);

    // Send input data to DFN
    primitive_finder->imageInput(*input);
    primitive_finder->primitiveInput(circle);

    // Run DFN
    primitive_finder->process();

    // Query output data from DFN
    const asn1SccVectorXdSequence& output = primitive_finder->primitivesOutput();

    // Check the DFN output vector is not empty
    BOOST_ASSERT(output.nCount > 0);
}

TEST_CASE( "Call to configure (BasicPrimitiveFinder) ", "[configure]" )
{
    // Prepare input data
    cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    FrameSharedConstPtr input = MatToFrameConverter().ConvertShared(gray);

    //Circle finder:
    std::string circle_string = "circle";
    asn1SccT_String circle;
    circle.nCount = circle_string.size();
    memcpy(circle.arr, circle_string.data(), circle_string.length());

    // Instantiate DFN
    std::unique_ptr<BasicPrimitiveFinder> primitive_finder (new BasicPrimitiveFinder);

    // Setup DFN
    primitive_finder->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PrimitiveFinder/BasicPrimitiveFinder.yaml");
    primitive_finder->configure();

    // Send input data to DFN
    primitive_finder->imageInput(*input);
    primitive_finder->primitiveInput(circle);

    // Run DFN
    primitive_finder->process();

    // Query output data from DFN
    const asn1SccVectorXdSequence& output = primitive_finder->primitivesOutput();

    // Check the DFN output vector is not empty
    BOOST_ASSERT(output.nCount > 0);
}

/** @} */
