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

            /**
            * Performs K-Means clustering on a 3C metric Image.
            * 1. Initializes clusters centroids randomly with K number of seeds
            * 2. Assigns each point to closest centroid
            * 3. replace centroids with new cluster's mean
            * 4. Computes distance between new and old centroid.
            *    if below tolerance, algorithm converged
            *    otherwise, back to step 2.
            * @inputs :
            *        3-channel Image including values in meters for X,Y,Z respectively
            * @outputs :
            *        Vector of K centroids
            * @params :
            *        - number of set clusters K
            *        - max iterations
            *        - error tolerance
            */

            class KMeansClustering : public ImageFilteringInterface {
            public:
                KMeansClustering();

                void configure() override;

                void process() override;

            private:

                struct Parameters {
                    int num_centers;        //  Number of clusters (minimum 2)
                    int max_iterations;     //  Maximum consecutive clustering before convergence
                    double tolerance;       //  Metric error below which clustering converged, to be compared with RMS of errors between cluster means and new centers.

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
