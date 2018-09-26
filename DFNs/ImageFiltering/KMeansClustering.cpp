
#include "KMeansClustering.hpp"

#include <Validators/Frame.hpp>
#include <Validators/Number.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


void print_uniques(const cv::Mat& mat) {

    std::set<uint8_t> unique_values;
    for (int row = 0; row < mat.rows; ++row) {
        for (int col = 0; col < mat.cols; ++col) {
            unique_values.insert(mat.at<uint8_t>(row, col));
        }
    }
}

namespace CDFF {
    namespace DFN {
        namespace ImageFiltering {
            const KMeansClustering::Parameters KMeansClustering::DefaultParameters = {};

            KMeansClustering::KMeansClustering() {
                _parametersHelper.AddParameter("KMeansClustering", "num_centers",
                                               _parameters.num_centers, DefaultParameters.num_centers);
                _parametersHelper.AddParameter<double>("KMeansClustering", "tolerance", _parameters.tolerance,
                                                       DefaultParameters.tolerance);
                _parametersHelper.AddParameter("KMeansClustering", "max_iterations",
                                               _parameters.max_iterations, DefaultParameters.max_iterations);
            }

            void KMeansClustering::configure() {
                _parametersHelper.ReadFile(configurationFilePath);
                ValidateParameters();
            }

            void KMeansClustering::process() {
                ValidateInputs(inImage);

                cv::Mat inputImage;

                // If the input has 3 channels then we need to discard the X and Y channels and
                // extract the Z channel otherwise we can simply convert the image to a cv::Mat
                if (inImage.data.channels == 1) {
                    inputImage = cv::Mat(inImage.data.rows, inImage.data.cols, CV_32FC1);
                    std::copy(inImage.data.data.arr, inImage.data.data.arr + inImage.data.data.nCount, inputImage.data);

                } else if (inImage.data.channels == 3) {
                    cv::Mat input_3channel(inImage.data.rows, inImage.data.cols, CV_32FC3);
                    std::copy(inImage.data.data.arr, inImage.data.data.arr + inImage.data.data.nCount,
                              input_3channel.data);

                    std::array<cv::Mat, 3> in_channels;
                    cv::split(input_3channel, in_channels.data());

                    inputImage = in_channels[2];
                } else {
                    ASSERT(false, "Source image should have 1 or 3 channels");
                }

                // On the first run we need to generate the seeds for the algorithm
                // If we reconfigure the DFN and change the number of centroids we should also
                // recompute a new set.
                if (_kmeans_centroids.size() != _parameters.num_centers)
                    _kmeans_centroids = PickCentroids(inputImage, static_cast<size_t>(_parameters.num_centers));

                cv::Mat clusters;
                std::vector<float> new_centroids, old_centroids = _kmeans_centroids;

                // Now we must look at the entire image, assign each pixel to the nearest cluster and
                // update the centroid of each cluster.

                double diff_squared = 0.0;
                size_t iteration_count = 0;

                do {
                    ++iteration_count;

                    clusters = ClusterPixels(inputImage, old_centroids);
                    new_centroids = UpdateClusters(inputImage, clusters);

                    diff_squared = 0.0;
                    for (size_t i = 0; i < old_centroids.size(); ++i) {
                        diff_squared += std::pow(old_centroids[i] - new_centroids[i], 2);
                    }

                    old_centroids = new_centroids;
                } while (diff_squared / _parameters.num_centers > _parameters.tolerance
                         && iteration_count < _parameters.max_iterations);

                _kmeans_centroids = new_centroids;
                outImage = *Converters::MatToFrameConverter().Convert(clusters);
            }


            void KMeansClustering::ValidateParameters() {
                Validators::Number::IsPositive(_parameters.num_centers);
                Validators::Number::IsPositive(_parameters.tolerance);
            }

            void KMeansClustering::ValidateInputs(const FrameWrapper::Frame &frame) {

            }

            cv::Mat
            KMeansClustering::ClusterPixels(const cv::Mat &inputImage, const std::vector<float> &kmeans_centroids) {
                ASSERT(_parameters.num_centers == kmeans_centroids.size(), "Wrong number of centroids");
                cv::Mat clusters = cv::Mat(inputImage.size(), CV_8UC1, cv::Scalar::all(0));

                for (int row = 0; row < inputImage.rows; ++row) {
                    for (int col = 0; col < inputImage.cols; ++col) {
                        const auto &this_pixel = inputImage.at<float>(row, col);

                        // 0 depth is impossible - These points should be skipped
                        if (this_pixel == 0.f)
                            continue;

                        // Assign the pixel to the "closest" centroid
                        auto min_norm = std::numeric_limits<float>::max();
                        for (size_t centroid_label = 1; centroid_label <= _parameters.num_centers; ++centroid_label) {
                            const auto centroid_distance = std::abs(this_pixel - kmeans_centroids[centroid_label]);
                            if (centroid_distance < min_norm) {
                                clusters.at<uint8_t>(row, col) = static_cast<uint8_t>(centroid_label);
                                min_norm = centroid_distance;
                            }
                        }
                    } // End for cols
                } // End for rows

                return clusters;
            }

            std::vector<float> KMeansClustering::UpdateClusters(const cv::Mat &inputImage, cv::Mat &clusters) {
                const auto num_clusters = static_cast<size_t>(_parameters.num_centers);

                // Compute the mean of each cluster - first count and sum the pixels then compute the mean
                std::vector<float> cluster_sums(num_clusters, 0.f);
                std::vector<unsigned int> cluster_counts(num_clusters, 0);

                for (int row = 0; row < inputImage.rows; ++row) {
                    for (int col = 0; col < inputImage.cols; ++col) {
                        cluster_sums[clusters.at<char>(row, col)] += inputImage.at<float>(row, col);
                        cluster_counts[clusters.at<char>(row, col)] += 1;
                    }
                }

                std::vector<float> cluster_means(num_clusters, 0.f);
                for (int i = 0; i < num_clusters; ++i) {
                    if (cluster_counts[i] != 0)
                        cluster_means[i] = cluster_sums[i] / cluster_counts[i];
                }

                // Check for empty clusters and replace them with new random seeds
                // If the mask was empty then cv::mean returns all zeroes.
                for (size_t i = 0; i < num_clusters; ++i) {
                    if (cluster_counts[i] == 0)
                        cluster_means[i] = PickCentroids(inputImage, 1)[0];
                }

                return cluster_means;
            }

            std::vector<float> KMeansClustering::PickCentroids(
                    const cv::Mat &srcImage, size_t num_points
            ) const {
                std::random_device rd;
                std::default_random_engine random_engine;
                std::uniform_int_distribution<int> centroid_picker(0, static_cast<int>(srcImage.total()));

                std::vector<float> centroids;
                for (int i = 0; i < num_points; ++i) {
                    centroids.emplace_back(
                            srcImage.at<float>(centroid_picker(random_engine))
                    );
                }

                return centroids;
            }
        }
    }
}
