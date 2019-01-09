/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOSLAM_STEREOSLAMORB_HPP
#define STEREOSLAM_STEREOSLAMORB_HPP

#include "StereoSlamInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include "liborbslam/System.h"

namespace CDFF
{
namespace DFN
{
namespace StereoSlam
{

/**
     * Visual Slam localization of the camera using a rectified stereo image pair input.
     * The underlying system is ORB-SLAM. For the full details on the SLAM implementation,
     * please see:  Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System
     * for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics, vol. 33,
     * no. 5, pp. 1255-1262, 2017.
     * https://arxiv.org/pdf/1610.06475.pdf
     *
     * @brief DFN which performs SLAM tracking using only a rectified stereo pair as input.
     * If tracking is successful, it outputs the current estimated camera pose expressed in the reference
     * frame of the first input image.
     *
     * @param settingsPath
     *        Path to the setting file to be passed to ORB-SLAM.
     *        The settings file contains operational parameters for the SLAM system, as well as camera calibration
     *        parameters. An example of such a settings file is available in Tests/ConfigurationFiles/DFNs/StereoSlam
     * @param vocabularyPath
     *        Path to the binary vocabulary file to be used by ORB-SLAM. This vocabulary consists in
     *        a pre-trained, serialized bag-of-words structure used to perform ORB matching for
     *        re-localisation in the SLAM system. The file should be available in your CDFF
     *        sources as ORBVoc.bin.
     * @param enableViewer
     *        Set to true to enable the viewer component, powered by Pangolin. The viewer allows the
     *        user to see the SLAM map, keyframes and trajectory in real-time. An output of detected
     *        features overlayed on the current image is also available. The viewer exposes some controls
     *        to the user, such as a way to reset the system, or switch into a localisation-only mode.
     * @param saveMap
     *        Set to true to save the SLAM map built during this execution into a serialized binary file.
     *        The saved SLAM map can be reloaded in a further run to perform Visual Map-based Localisation.
     * @param mapFile
     *        Path to the SLAM map file generated during a previous run. If the file already exists, the map
     *        will be loaded at execution, and the system will use it to perform relocalisation when
     *        and area is revisited. If the saveMap parameter is set to true, the map generated during
     *        the current run will be saved to this path.
     */
class StereoSlamOrb : public StereoSlamInterface
{
public:

    StereoSlamOrb();
    virtual ~StereoSlamOrb();

    virtual void configure();
    virtual void process();

private:
    struct StereoSlamOrbOptionsSet
    {
        std::string settingsPath;
        std::string vocabularyPath;
        bool enableViewer;
        bool saveMap;
        std::string mapFile;
    };

    Helpers::ParametersListHelper parametersHelper;
    StereoSlamOrbOptionsSet parameters;
    static const StereoSlamOrbOptionsSet DEFAULT_PARAMETERS;

    void ValidateParameters();
    void ValidateInputs(const asn1SccFramePair& pair);

    ORB_SLAM2::System *slam;
};
}
}
}

#endif // STEREOSLAM_STEREOSLAMORB_HPP

/** @} */
