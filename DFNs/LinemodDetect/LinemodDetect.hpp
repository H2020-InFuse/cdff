/**
 * @addtogroup DFNs
 * @{
 */

#ifndef LINEMODDETECT_LINEMODDETECT_HPP
#define LINEMODDETECT_LINEMODDETECT_HPP

#include "LinemodDetectInterface.hpp"

#include <Converters/MatToTransform3DConverter.hpp>
#include "Helpers/ParametersListHelper.hpp"
#include "linemod-wrapper/LinemodDetectImpl.hpp"


namespace CDFF
{
namespace DFN
{
namespace LinemodDetect
{
    class LinemodDetect : public LinemodDetectInterface
    {
        public:

            LinemodDetect();
            virtual ~LinemodDetect();

            virtual void configure();
            virtual void process();

            struct LinemodDetectParams
            {
                int T_level0;
                int T_level1;
                float matchingThreshold;
                double fx;
                double fy;
                double cx;
                double cy;
                std::string cadObjectName;
                bool useDepthModality;
                bool resizeVGA;
                bool displayResult;
            };

            LinemodDetectParams parameters;
            bool getDetection(float& similarity_, cv::Rect& detection_, std::string& class_id_, int& template_id_,
                              cv::Vec3d& vec_R_, cv::Vec3d& vec_T_, cv::Mat& cameraPose_);

    private:
            void ValidateParameters();
            cv::Mat convertToCameraPose(const cv::Vec3d& vec_R, const cv::Vec3d& vec_T, const cv::Rect& detection);

            LinemodBasedPoseDetector linemodDetector;
            Helpers::ParametersListHelper parametersHelper;
            static const LinemodDetectParams DEFAULT_PARAMETERS;
            bool retDetection;
            float similarity;
            cv::Rect detection;
            std::string class_id;
            int template_id;
            cv::Vec3d detection_R;
            cv::Vec3d detection_T;
            cv::Mat cameraPose;
            Converters::MatToPose3DConverter matToPose3D;
    };
}
}
}

#endif // LINEMODDETECT_LINEMODDETECT_HPP

/** @} */
