/*!
 * @addtogroup DFPCsTest
 *
 * Unit Test for the DFPC WheeledRobotTracker.
 *
 *
 * @{
 */

#include <catch.hpp>
#include <ModelBasedTracker/WheeledRobotTracker.hpp>
#include <Errors/Assert.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Types/CPP/Frame.hpp>

using namespace CDFF::DFPC::ModelBasedTracker;
using namespace FrameWrapper;
using namespace BaseTypesWrapper;
using namespace Array3DWrapper;

TEST_CASE( "Call to setup (WheeledRobotTracker)", "[setup]" )
{
    std::unique_ptr<WheeledRobotTracker> tracker( new WheeledRobotTracker() );
    tracker->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedTracker/DfpcModelBasedTracker_conf01.yaml");
    tracker->setup();
}

TEST_CASE( "Call to run (WheeledRobotTracker)", "[run]" )
{
    // Prepare input data
    cv::FileStorage file_reader("../tests/Data/depthImg_Mana.yml.gz", cv::FileStorage::READ);
    ASSERT(file_reader.isOpened(), "Unable to input image file");
    cv::Mat input;
    file_reader.getFirstTopLevelNode() >> input;
    ASSERT(!input.empty(), "Something went wrong when reading the image");

    FrameSharedPtr depth_img = NewSharedFrame();
    Initialize(*depth_img);
    depth_img->metadata.mode = FrameMode::asn1Sccmode_XYZ;
    depth_img->metadata.status = FrameStatus::asn1Sccstatus_VALID;
    depth_img->data.rows = static_cast<T_UInt16>(input.rows);
    depth_img->data.cols = static_cast<T_UInt16>(input.cols);
    depth_img->data.rowSize = static_cast<T_UInt32>(input.step[0]);
    depth_img->data.channels = 3;
    depth_img->data.depth = Array3DDepth::asn1Sccdepth_32S;

    const size_t datasize = input.total() * input.elemSize();
    assert(sizeof(depth_img->data.data.arr) > datasize);
    std::copy(input.data, input.data + datasize, depth_img->data.data.arr);

    depth_img->data.data.nCount = static_cast<int>(input.total() * input.elemSize());

    cv::Mat img = cv::imread("../tests/Data/Images/primitive_matching/test_images/robot.jpg", cv::IMREAD_GRAYSCALE);
    FrameWrapper::FrameConstPtr input_img = Converters::MatToFrameConverter().Convert(img);

    std::string robot_name = "MANA";
    asn1SccT_String robot_name_asn1;
    robot_name_asn1.nCount = robot_name.size();
    memcpy(robot_name_asn1.arr, robot_name.data(), robot_name.length());

    std::unique_ptr<WheeledRobotTracker> tracker( new WheeledRobotTracker() );
    tracker->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedTracker/DfpcModelBasedTracker_conf01.yaml");
    tracker->setup();

    tracker->imageInput(*input_img);
    tracker->robotNameInput(robot_name_asn1);
    tracker->depthInput(*depth_img);

    tracker->run();

    const asn1SccPose& outPose = tracker->poseOutput();
}

/** @} */
