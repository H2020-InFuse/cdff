/**
 * @addtogroup DFNs
 * @{
 */
#include "StereoSlamOrb.hpp"

#include "Errors/Assert.hpp"
#include "Types/CPP/BaseTypes.hpp"

#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>

namespace CDFF
{
namespace DFN
{
namespace StereoSlam
{

StereoSlamOrb::StereoSlamOrb()
{
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<std::string>("GeneralParameters", "SettingsPath", parameters.settingsPath, DEFAULT_PARAMETERS.settingsPath);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "VocabularyPath", parameters.vocabularyPath, DEFAULT_PARAMETERS.vocabularyPath);
    parametersHelper.AddParameter<bool>("GeneralParameters", "EnableViewer", parameters.enableViewer, DEFAULT_PARAMETERS.enableViewer);
    parametersHelper.AddParameter<bool>("GeneralParameters", "SaveMap", parameters.saveMap, DEFAULT_PARAMETERS.saveMap);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "MapFile", parameters.mapFile, DEFAULT_PARAMETERS.mapFile);

    configurationFilePath = "";
    slam = nullptr;

    //Initialize output structure
    asn1SccTransformWithCovariance_Initialize(&outPose);
}

StereoSlamOrb::~StereoSlamOrb()
{
    // Shutdown the slam system
    slam->Shutdown();

    if( slam != nullptr )
    {
        delete slam;
        slam = nullptr;
    }
}

void StereoSlamOrb::configure()
{
    parametersHelper.ReadFile(configurationFilePath);
    ValidateParameters();

    // Initialize slam system
    slam = new ORB_SLAM2::System(parameters.vocabularyPath.c_str(),parameters.settingsPath.c_str(),ORB_SLAM2::System::STEREO,parameters.enableViewer,parameters.saveMap);
}

void StereoSlamOrb::process()
{
    ASSERT(slam != nullptr, "StereoSlamorb: Error, slam system not initialized");

    // Retrieve data and convert to cv matrices
    cv::Mat left(inImagePair.left.data.rows, inImagePair.left.data.cols, CV_MAKETYPE((int)(inImagePair.left.data.depth), inImagePair.left.data.channels), inImagePair.left.data.data.arr, inImagePair.left.data.rowSize);
    cv::Mat right(inImagePair.right.data.rows, inImagePair.right.data.cols, CV_MAKETYPE((int)(inImagePair.right.data.depth), inImagePair.right.data.channels), inImagePair.right.data.data.arr, inImagePair.right.data.rowSize);

    // Make sure the image mode is correct
    if( (inImagePair.left.metadata.mode != 0) && (inImagePair.right.metadata.mode != 1) )
    {
        PRINT_WARNING("Image mode is different of RGB or BGR");
    }
    else
    {
        // Perform slam iteration
        cv::Mat cvPose;
        cvPose = slam->TrackStereo(left, right, inImagePair.left.metadata.timeStamp.microseconds *1.0e-6);
        if( !cvPose.empty() )
        {
            // Set the pose metadata to the input metadata
            BaseTypesWrapper::CopyString("StereoSlamOrb", outPose.metadata.producerId);
            outPose.metadata.childFrameId = inImagePair.left.extrinsic.pose_robotFrame_sensorFrame.metadata.childFrameId;
            BaseTypesWrapper::CopyString("InitialCamera", outPose.metadata.parentFrameId);
            outPose.metadata.childTime = inImagePair.left.metadata.timeStamp;
            outPose.metadata.parentTime = inImagePair.left.metadata.timeStamp;

            // Set the pose output data to the estimation
            Eigen::Matrix4d tmp;
            cv::cv2eigen(cvPose, tmp);
            Eigen::Isometry3d iso(tmp);
            outPose.data.translation.arr[0] = iso.translation()[0];
            outPose.data.translation.arr[1] = iso.translation()[1];
            outPose.data.translation.arr[2] = iso.translation()[2];
            Eigen::Quaterniond quaternion(iso.rotation());
            outPose.data.orientation.arr[0] = quaternion.coeffs()[0];
            outPose.data.orientation.arr[1] = quaternion.coeffs()[1];
            outPose.data.orientation.arr[2] = quaternion.coeffs()[2];
            outPose.data.orientation.arr[3] = quaternion.coeffs()[3];
        }
    }
}

const StereoSlamOrb::StereoSlamOrbOptionsSet StereoSlamOrb::DEFAULT_PARAMETERS =
{
    /*.settingPath =*/ "",
    /*.vocabularyPath =*/ "",
    /*.enableViewer =*/ false,
    /*.saveMap =*/ false,
    /*.mapFile =*/ ""
};

void StereoSlamOrb::ValidateParameters()
{
    ASSERT(parameters.settingsPath.size() > 0, "StereoSlamOrb Configuration Error: Orb-Slam settings file path must be specified");
    ASSERT(parameters.vocabularyPath.size() > 0, "StereoSlamOrb Configuration Error: Orb-Slam vocabulary file path must be specified");
    if(parameters.saveMap)
    {
        ASSERT(parameters.mapFile.size() > 0, "StereoSlamOrb Configuration Error: A path to the previously generated map file must be specified");
    }
}

void StereoSlamOrb::ValidateInputs(const asn1SccFramePair& pair)
{
    //TODO
}

}
}
}

/** @} */
