
#include <iostream>

#include "KMeansClustering.h"

#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


namespace Validators {
    namespace Number {
        template <class NumberType>
        void GreaterThan(NumberType value, NumberType min_val) {
            ASSERT(value > min_val, "Validators/Number - Should be greater than specified value");
        }
    }

    namespace Frame {
        void NotEmpty(const FrameWrapper::Frame& frame) {
            ASSERT(frame.data.cols > 0 && frame.data.rows > 0,
                   "Validators/Frame - Image should not be empty");
        }

        void HasFormatIn(const FrameWrapper::Frame &frame, const std::vector<FrameWrapper::FrameMode>& frameModes) {
            const size_t mode_count = frameModes.size();
            for (size_t idx = 0; idx < mode_count; ++idx) {
                    if (frame.metadata.mode == frameModes[idx])
                    {
                            return;
                    }
            }

            ASSERT(false, "Frame Mode not in approved list");
        }
    }
}

namespace CDFF {
    namespace DFN {
        const KMeansClustering::Parameters KMeansClustering::DefaultParameters = {};

        KMeansClustering::KMeansClustering() {
            _parametersHelper.AddParameter("KMeansClustering", "num_centers",
                    _parameters.num_centers, DefaultParameters.num_centers);
            _parametersHelper.AddParameter<double>("KMeansClustering", "tolerance", _parameters.tolerance, 1.0);
            _parametersHelper.AddParameter("KMeansClustering", "max_iterations",
                    _parameters.max_iterations, DefaultParameters.max_iterations);
        }

        void KMeansClustering::configure() {
            _parametersHelper.ReadFile(configurationFilePath);
            ValidateParameters();
        }

        void KMeansClustering::process() {
            ValidateInputs(inImage);

             cv::Mat inputImage(inImage.data.rows, inImage.data.cols, CV_32FC3);
             std::copy(inImage.data.data.arr, inImage.data.data.arr + inImage.data.data.nCount, inputImage.data);

            if (_kmeans_centers.empty())
                _kmeans_centers = GenerateRandomCenters(inputImage, static_cast<size_t>(_parameters.num_centers));

            cv::Mat clusters;
            std::vector<cv::Vec3f> new_kmeans_centers;

            double mean_distance = std::numeric_limits<float>::max();
            size_t iteration_count = 0;
            while (mean_distance > _parameters.tolerance && iteration_count < _parameters.max_iterations) {
                 clusters = ClusterPixels(inputImage, _kmeans_centers);
                new_kmeans_centers = UpdateClusters(inputImage, clusters);

                double distance = 0.0;
                for (size_t i = 0; i < _parameters.num_centers; ++i) {
                    distance += cv::norm(_kmeans_centers[i], new_kmeans_centers[i], cv::NORM_L2);
                }

                mean_distance = distance / _parameters.num_centers;
                _kmeans_centers = new_kmeans_centers;
                ++iteration_count;
            }

            outImage = *Converters::MatToFrameConverter().Convert(clusters);
        }

        void KMeansClustering::ValidateParameters() {
            Validators::Number::GreaterThan(_parameters.num_centers, 0);
            Validators::Number::GreaterThan(_parameters.tolerance, 0.);
        }

        void KMeansClustering::ValidateInputs(const FrameWrapper::Frame &frame) {

        }

        cv::Mat KMeansClustering::ClusterPixels(const cv::Mat &inputImage, const std::vector<cv::Vec3f> &kmeans_centers) {
            ASSERT(_parameters.num_centers == kmeans_centers.size(), "Wrong number of centers");
            cv::Mat clusters = cv::Mat(inputImage.size(), CV_8UC1, cv::Scalar::all(0));

            for (int row = 0; row < inputImage.rows; ++row) {
                for (int col = 0; col < inputImage.cols; ++col) {
                    const auto& this_pixel = inputImage.at<cv::Vec3f>(row, col);

                    // 0 depth is impossible - These points should be skipped
                    if (this_pixel[2] == 0.f)
                        continue;

                    auto min_norm = std::numeric_limits<float>::max();
                    for (size_t i = 0; i < _parameters.num_centers; ++i) {
                        // Assign the pixel to the "closest" (L2 Norm) center
                        const auto this_norm = norm(this_pixel, kmeans_centers[i], cv::NORM_L2);
//                        const auto this_norm = std::sqrt( std::pow(this_pixel[0], 2) + std::pow(kmeans_centers[0][2], 2) );
                        if (this_norm < min_norm) {
                            clusters.at<uint8_t>(row, col) = static_cast<uint8_t>(i +1);
                            min_norm = static_cast<float>(this_norm);
                        }
                    }
                }
            }

            return clusters;
        }

        std::vector<cv::Vec3f> KMeansClustering::UpdateClusters(const cv::Mat &inputImage, cv::Mat &clusters) {
            const int num_clusters = _parameters.num_centers;
            std::vector<cv::Vec3f> cluster_sums(num_clusters, cv::Vec3f::all(0.f));
            std::vector<unsigned int> cluster_counts(num_clusters, 0);

            for (int row = 0; row < inputImage.rows; ++row) {
                for (int col = 0; col < inputImage.cols; ++col) {
                    for (int cluster = 0; cluster < num_clusters; ++cluster) {
                        cluster_sums[clusters.at<char>(row, col)] += inputImage.at<cv::Vec3f>(row, col);
                        cluster_counts[clusters.at<char>(row, col)] += 1;
                    }
                }
            }

            std::vector<cv::Vec3f> cluster_means(num_clusters, cv::Vec3f::all(0.f));
            for (int i = 0; i < num_clusters; ++i) {
                if (cluster_counts[i] != 0)
                    cluster_means[i] =  cv::Vec3f(
                        cluster_sums[i][0] / cluster_counts[i],
                        cluster_sums[i][1] / cluster_counts[i],
                        cluster_sums[i][2] / cluster_counts[i]
                    );
            }

            // Check for empty clusters and replace them with new random seeds
            // If the mask was empty then cv::mean returns all zeroes.
            for (size_t i = 0; i < num_clusters; ++i) {
                if (cluster_counts[i] == 0)
                    cluster_means[i] = GenerateRandomCenters(inputImage, 1)[0];
            }

            return cluster_means;
        }

        std::vector<cv::Vec3f> KMeansClustering::GenerateRandomCenters(
                const cv::Mat &srcImage, const size_t num_points
        ) const {

            std::vector<cv::Mat> channels(3);
            cv::split(srcImage, channels);

            cv::Mat channel_x = channels[0], channel_y = channels[1], channel_z = channels[2];

            double min_x, max_x, min_y, max_y, min_z, max_z;
            cv::minMaxLoc(channel_x, &min_x, &max_x);
            cv::minMaxLoc(channel_y, &min_y, &max_y);
            cv::minMaxLoc(channel_z, &min_z, &max_z);

            std::random_device rd;
            std::default_random_engine random_engine;
            std::uniform_real_distribution<double> x_range(min_x, max_x);
            std::uniform_real_distribution<double> y_range(min_y, max_y);
            std::uniform_real_distribution<double> z_range(min_z, max_z);

            std::vector<cv::Vec3f> centers;
            for (int i = 0; i < num_points; ++i) {
                centers.emplace_back(
                    static_cast<float>(x_range(random_engine)),
                    static_cast<float>(y_range(random_engine)),
                    static_cast<float>(z_range(random_engine))
                );

                if (centers.back()[1] == 0.0f or centers.back()[2] == 0.0f) {
                    std::cout << "Warn" << std::endl;
                }
            }

            return centers;
        }
    }
}