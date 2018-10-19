//
// Created by tch on 04/09/18.
//

#ifndef IMAGEFILTERING_KMEANSCLUSTERING_HPP
#define IMAGEFILTERING_KMEANSCLUSTERING_HPP

#include <opencv2/core.hpp>

#include <Types/CPP/Frame.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include "ImageFilteringInterface.hpp"

namespace CDFF {
    namespace DFN {
        namespace ImageFiltering {

            class KMeansClustering : public ImageFilteringInterface {
            public:
                KMeansClustering();

                void configure() override;

                void process() override;

            private:

                struct Parameters {
                    int num_centers = 5;
                    int max_iterations = 20;
                    double tolerance = 1e-2;
                };
                static const Parameters DefaultParameters;

                void ValidateParameters();

                void ValidateInputs(const FrameWrapper::Frame &frame);

                cv::Mat ClusterPixels(const cv::Mat &inputImage, const std::vector<float> &kmeans_centroids);

                std::vector<float> UpdateClusters(const cv::Mat &inputImage, cv::Mat &clusters);

                std::vector<float> PickCentroids(const cv::Mat &srcImage, size_t num_points = 1) const;

                Parameters _parameters;
                Helpers::ParametersListHelper _parametersHelper;

                /* State Variables */
                std::vector<float> _kmeans_centroids;
            };

        }
    }
}
#endif //IMAGEFILTERING_KMEANSCLUSTERING_HPP
