/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HuInvariants.cpp
 * @date 07/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN Hu Invariants.
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
#include <catch.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <PrimitiveMatching/HuInvariants.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <StringArrayToStdVectorOfStringsConverter.hpp>
#include <StdVectorOfStringsToStringArrayConverter.hpp>


using namespace CDFF::DFN::PrimitiveMatching;
using namespace Converters;
using namespace FrameWrapper;

using namespace CDFF::DFN::PrimitiveMatching;
using namespace Converters;
using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

//=====================================================================================================================
TEST_CASE( "HuInvariants - check rotated star is correctly matched" )
{
    // Instantiate DFN
    HuInvariants* hu_invariants = new HuInvariants();

    // Setup DFN
    hu_invariants->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PrimitivesMatching/HuInvariants.yaml");
    hu_invariants->configure();

    std::string image_file_path = "../tests/Data/Images/primitive_matching/test_images/rotated_star.jpg";

    MatToFrameConverter converter;
    cv::Mat cv_image = cv::imread(image_file_path, cv::IMREAD_COLOR);
    FrameConstPtr input_image = converter.Convert(cv_image);

    std::vector<std::string> string_array{"star", "circle"};
    BaseTypesWrapper::asn1SccT_StringArray primitive_array = StdVectorOfStringsToStringArrayConverter().Convert(string_array);
    hu_invariants->primitiveArrayInput(primitive_array);

    // Send input data to DFN
    hu_invariants->frameInput(*input_image);

    // Run DFN
    hu_invariants->process();

    std::vector<std::string> ordered_primitives = Converters::StringArrayToStdVectorOfStringsConverter().Convert(hu_invariants->primitivesMatchedOutput());
    CHECK( ordered_primitives[0] == "star");
}

//=====================================================================================================================
TEST_CASE( "HuInvariants - check parallelogram is correctly matched" )
{
    // Instantiate DFN
    HuInvariants* hu_invariants = new HuInvariants();

    // Setup DFN
    hu_invariants->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PrimitivesMatching/HuInvariants.yaml");
    hu_invariants->configure();

    std::string image_file_path = "../tests/Data/Images/primitive_matching/test_images/parallelogram.jpg";

    MatToFrameConverter converter;
    cv::Mat cv_image = cv::imread(image_file_path, cv::IMREAD_COLOR);
    FrameConstPtr input_image = converter.Convert(cv_image);

    // Send input data to DFN
    hu_invariants->frameInput(*input_image);

    std::vector<std::string> string_array{"rectangle", "circle"};
    BaseTypesWrapper::asn1SccT_StringArray primitive_array = StdVectorOfStringsToStringArrayConverter().Convert(string_array);
    hu_invariants->primitiveArrayInput(primitive_array);

    // Run DFN
    hu_invariants->process();

    std::vector<std::string> ordered_primitives = Converters::StringArrayToStdVectorOfStringsConverter().Convert(hu_invariants->primitivesMatchedOutput());
    CHECK( ordered_primitives[0] == "rectangle");
}

//=====================================================================================================================
TEST_CASE( "HuInvariants - check star is correctly matched" )
{
    // Instantiate DFN
    HuInvariants* hu_invariants = new HuInvariants();

    // Setup DFN
    hu_invariants->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PrimitivesMatching/HuInvariants.yaml");
    hu_invariants->configure();

    std::string image_file_path = "../tests/Data/Images/primitive_matching/test_images/shapes.jpg";

    MatToFrameConverter converter;
    cv::Mat cv_image = cv::imread(image_file_path, cv::IMREAD_COLOR);
    FrameConstPtr input_image = converter.Convert(cv_image);

    // Send input data to DFN
    hu_invariants->frameInput(*input_image);

    std::vector<std::string> string_array{"star", "circle"};
    BaseTypesWrapper::asn1SccT_StringArray primitive_array = StdVectorOfStringsToStringArrayConverter().Convert(string_array);
    hu_invariants->primitiveArrayInput(primitive_array);

    // Run DFN
    hu_invariants->process();

    std::vector<std::string> ordered_primitives = Converters::StringArrayToStdVectorOfStringsConverter().Convert(hu_invariants->primitivesMatchedOutput());
    CHECK( ordered_primitives[0] == "star");
}

/** @} */
