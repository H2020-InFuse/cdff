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
#include <WeightingExpert/PoseWeighting.hpp>


using namespace CDFF::DFN::WeightingExpert;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (PoseWeighting)", "[process]" )
{
    // Prepare input data
    asn1SccPosesSequence poses;
    poses.nCount = 2;
    poses.arr[0].pos.arr[0] = 1;
    poses.arr[0].pos.arr[1] = 1;
    poses.arr[0].pos.arr[2] = 1;
    poses.arr[0].orient.arr[0] = 1;
    poses.arr[0].orient.arr[1] = 1;
    poses.arr[0].orient.arr[2] = 1;
    poses.arr[0].orient.arr[3] = 1;

    poses.arr[1].pos.arr[0] = 2;
    poses.arr[1].pos.arr[1] = 2;
    poses.arr[1].pos.arr[2] = 2;
    poses.arr[1].orient.arr[0] = 2;
    poses.arr[1].orient.arr[1] = 2;
    poses.arr[1].orient.arr[2] = 2;
    poses.arr[1].orient.arr[3] = 2;

    // Instantiate DFN
    std::unique_ptr<PoseWeighting> pose_weighting (new PoseWeighting);

    // Send input data to DFN
    pose_weighting->posesInput(poses);

    // Run DFN more than the number of stabilizing frames (default = 4)
    for(unsigned int i = 0; i < 5; i++)
    {
        pose_weighting->process();
    }

    // Query output data from DFN
    const asn1SccPose& output = pose_weighting->poseOutput();

    // Check the DFN output pose is different from init pose (all 0s)
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
}

TEST_CASE( "Call to configure (PoseWeighting) ", "[configure]" )
{
    // Prepare input data
    asn1SccPosesSequence poses;
    poses.nCount = 2;
    poses.arr[0].pos.arr[0] = 1;
    poses.arr[0].pos.arr[1] = 1;
    poses.arr[0].pos.arr[2] = 1;
    poses.arr[0].orient.arr[0] = 1;
    poses.arr[0].orient.arr[1] = 1;
    poses.arr[0].orient.arr[2] = 1;
    poses.arr[0].orient.arr[3] = 1;

    poses.arr[1].pos.arr[0] = 2;
    poses.arr[1].pos.arr[1] = 2;
    poses.arr[1].pos.arr[2] = 2;
    poses.arr[1].orient.arr[0] = 2;
    poses.arr[1].orient.arr[1] = 2;
    poses.arr[1].orient.arr[2] = 2;
    poses.arr[1].orient.arr[3] = 2;

    // Instantiate DFN
    std::unique_ptr<PoseWeighting> pose_weighting (new PoseWeighting);

    // Setup DFN
    pose_weighting->setConfigurationFile("../tests/ConfigurationFiles/DFNs/WeightingExpert/PoseWeighting.yaml");
    pose_weighting->configure();

    // Send input data to DFN
    pose_weighting->posesInput(poses);

    // Run DFN more than the number of stabilizing frames (default = 4)
    for(unsigned int i = 0; i < 5; i++)
    {
        pose_weighting->process();
    }

    // Query output data from DFN
    const asn1SccPose& output = pose_weighting->poseOutput();

    // Check the DFN output pose is different from init pose (all 0s)
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.pos.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
    BOOST_ASSERT(output.orient.arr[0] != 0);
}

/** @} */