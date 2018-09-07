//
// Created by tch on 04/09/18.
//

#ifndef CDFF_KMEANSCLUSTERING_H
#define CDFF_KMEANSCLUSTERING_H

#include <opencv2/core.hpp>

#include <Types/CPP/Frame.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include "ImageFilteringInterface.hpp"

namespace CDFF {
    namespace DFN {

        class KMeansClustering: public ImageFilteringInterface {
            struct Parameters {
                int num_centers = 5;
                int max_iterations = 20;
                double tolerance = 1e-2;
            };

        public:
            static const Parameters DefaultParameters;

            KMeansClustering();

            void configure() override;

            void process() override;

        private:

            void ValidateParameters();

            void ValidateInputs(const FrameWrapper::Frame &frame);

            cv::Mat ClusterPixels(const cv::Mat &inputImage, const std::vector<cv::Vec3f> &kmeans_centers);

            std::vector<cv::Vec3f> UpdateClusters(const cv::Mat &inputImage, cv::Mat &clusters);

            std::vector<cv::Vec3f> GenerateRandomCenters(const cv::Mat &srcImage, const size_t num_points = 1) const;

            Parameters _parameters;
            Helpers::ParametersListHelper _parametersHelper;

            /* State Variables */
            std::vector<cv::Vec3f> _kmeans_centers;
        };

    }
}

#endif //CDFF_KMEANSCLUSTERING_H
