/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGERECTIFICATION_IMAGERECTIFICATION_HPP
#define IMAGERECTIFICATION_IMAGERECTIFICATION_HPP

#include "ImageRectificationInterface.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageRectification
{
    /**
     * @brief Implementation of the rectification process provided by OpenCV
     * See OpenCV's documentation on cv::remap() and cv::initUndistortRectifyMap() for more information
     * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#gab75ef31ce5cdfb5c44b6da5f3b908ea4
     * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
     */
    class ImageRectification : public ImageRectificationInterface
    {
        public:

            ImageRectification();
            virtual ~ImageRectification();

            virtual void configure();
            virtual void process();

            struct ImageRectificationParams
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
                 * @brief Principal point centering
                 * Indicates whether in the new camera matrix the principal point should be at the image center or not.
                 * By default, the principal point is chosen to best fit a subset of the source image (determined by the scaling parameter) to the corrected image.
                 */
                bool centerPrincipalPoint;

                /**
                 * @brief Set to true to use the fisheye camera projection model.
                 * Otherwise the pinhole projection model will be used.
                 */
                bool fisheye;
            };

            Helpers::ParametersListHelper parametersHelper;
            ImageRectificationParams parameters;
            static const ImageRectificationParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            std::string _sensorId;
            int _xratio;
            int _yratio;
            double _scaling;
            bool _centerPrincipalPoint;
            bool _fisheye;

            cv::Mat _mapx;
            cv::Mat _mapy;

            cv::Mat _newCameraMatrix;
    };

}
}
}

#endif // IMAGERECTIFICATION_IMAGERECTIFICATION_HPP

/** @} */
