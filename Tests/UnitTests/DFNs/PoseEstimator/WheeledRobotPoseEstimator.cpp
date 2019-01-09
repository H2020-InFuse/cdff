/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Pose Estimator.
 *
 *
 * @{
 */

#include <catch.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Converters/MatToFrameConverter.hpp>
#include <PoseEstimator/WheeledRobotPoseEstimator.hpp>

using namespace CDFF::DFN::PoseEstimator;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (WheeledRobotPoseEstimator)", "[process]" )
{
    // Prepare input data
    cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
    cv::Mat gray_cv_image;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

    asn1SccFrame inImage = MatToFrameConverter().Convert(gray);
    asn1SccFrame inDepth = MatToFrameConverter().Convert(gray);
    asn1SccVectorXdSequence inPrimitives = MatToFrameConverter().Convert(gray);
    asn1SccFrame frame_without_background = MatToFrameConverter().Convert(gray);

    // Instantiate DFN
    std::unique_ptr<WheeledRobotPoseEstimator> m_pose_estimator (new WheeledRobotPoseEstimator);

    // Send input data to DFN
    m_pose_estimator->imageInput(inImage);
    m_pose_estimator->depthInput(inDepth);
    m_pose_estimator->primitivesInput(inPrimitives);
    m_pose_estimator->imageInput(frame_without_background);

    // Run DFN more than the number of stabilizing frames (default = 4)
    for(unsigned int i = 0; i < 5; i++)
    {
        m_pose_estimator->process();
    }

    // Query output data from DFN
    const asn1SccPosesSequence & output = m_pose_estimator->posesOutput();

}

TEST_CASE( "Call to configure (WheeledRobotPoseEstimator) ", "[configure]" )
{
    // Instantiate DFN
    std::unique_ptr<WheeledRobotPoseEstimator> m_pose_estimator (new WheeledRobotPoseEstimator);

    // Setup DFN
    m_pose_estimator->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PoseEstimator/WheeledRobotPoseEstimator.yaml");
    m_pose_estimator->configure();

}

/** @} */
