/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECTIFICATION_STEREORECTIFICATION_HPP
#define STEREORECTIFICATION_STEREORECTIFICATION_HPP

#include "StereoRectificationInterface.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoRectification
{
    /**
     * @brief Implementation of the stereo rectification process provided by OpenCV
     * See OpenCV's documentation on cv::remap(), cv::initUndistortRectifyMap() and cv::stereoRectify() for more information
     * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#gab75ef31ce5cdfb5c44b6da5f3b908ea4
     * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
     * https://docs.opencv.org/3.1.0/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
     */
    class StereoRectification : public StereoRectificationInterface
    {
        public:

            StereoRectification();
            virtual ~StereoRectification();

            virtual void configure();
            virtual void process();

            struct StereoRectificationParams
            {
                /**
                 * @brief Degradation ratio to be applied over the x-axis
                 */
                int xratio;

                /**
                 * @brief Degradation ratio to be applied over the y-axis
                 */
                int yratio;

                /**
                 * @brief Free scaling parameter
                 * If it is -1, the function performs the default scaling.
                 * Otherwise, the parameter should be between 0 and 1.
                 * Scaling=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification).
                 * Scaling=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras are retained in the rectified images (no source image pixels are lost).
                 * Obviously, any intermediate value yields an intermediate result between those two extreme cases.
                 */
                double scaling;

                /**
                 * @brief Set to true to use the fisheye camera projection model.
                 * Otherwise the pinhole projection model will be used.
                 */
                bool fisheye;

                /**
                 * @brief Path to the calibration file
                 * The calibration file should be named "sensorIDLeft-sensorIDRight.yml"
                 * If this file is located in "/path/to/calibration/sensorIDLeft-sensorIDRight.yaml",
                 * then this parameter should be "/path/to/calibration"
                 */
                std::string calibrationFilePath;
            };

            Helpers::ParametersListHelper parametersHelper;
            StereoRectificationParams parameters;
            static const StereoRectificationParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            std::string _sensorIdLeft;
            std::string _sensorIdRight;
            std::string _calibrationFilePath;
            int _xratio;
            int _yratio;
            double _scaling;
            bool _centerPrincipalPoint;
            bool _fisheye;
            bool _initialized;

            cv::Mat _lmapx;
            cv::Mat _lmapy;
            cv::Mat _rmapx;
            cv::Mat _rmapy;

            cv::Mat1d _PLeft;
            cv::Mat1d _PRight;
            double _baseline;
    };
}
}
}

#endif // STEREORECTIFICATION_STEREORECTIFICATION_HPP

/** @} */
