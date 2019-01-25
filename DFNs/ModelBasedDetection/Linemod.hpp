/**
 * @addtogroup DFNs
 * @{
 */

#ifndef MODELBASEDDETECTION_LINEMOD_HPP
#define MODELBASEDDETECTION_LINEMOD_HPP

#include "ModelBasedDetectionInterface.hpp"

#include <Converters/MatToTransform3DConverter.hpp>
#include "Helpers/ParametersListHelper.hpp"
#include "linemod-wrapper/LinemodImpl.hpp"


namespace CDFF
{
namespace DFN
{
namespace ModelBasedDetection
{
    class Linemod : public ModelBasedDetectionInterface
    {
        public:

            Linemod();
            virtual ~Linemod();

            virtual void configure();
            virtual void process();

            struct LinemodParams
            {
                int T_level0;
                int T_level1;
                float matchingThreshold;
                std::string cadObjectName;
                bool useDepthModality;
                bool resizeVGA;
            };

            LinemodParams parameters;
            bool getDetection(float& similarity_, cv::Rect& detection_, std::string& class_id_, int& template_id_,
                              cv::Vec3d& vec_R_, cv::Vec3d& vec_T_, cv::Mat& cameraPose_);

    private:
            void ValidateParameters();
            cv::Mat convertToCameraPose(const cv::Vec3d& vec_R, const cv::Vec3d& vec_T);

            Helpers::ParametersListHelper parametersHelper;
            static const LinemodParams DEFAULT_PARAMETERS;
            LinemodBasedPoseDetector linemodDetector;
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

#endif // MODELBASEDDETECTION_LINEMOD_HPP

/** @} */
