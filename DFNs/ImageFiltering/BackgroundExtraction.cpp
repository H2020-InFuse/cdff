/**
 * @addtogroup DFNs
 * @{
 */

#include <opencv2/imgproc.hpp>

#include <Types/C/Frame.h>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>

#include "Validators/Frame.hpp"
#include "BackgroundExtraction.hpp"

using namespace FrameWrapper;

namespace CDFF {
    namespace DFN {
        namespace ImageFiltering {

            const BackgroundExtraction::Parameters BackgroundExtraction::DefaultParameters = {};


            cv::Point2i getPointFromNeighbourhoodOf(const cv::Point2i &pt, const cv::Size &size) {
                std::random_device rand;
                std::default_random_engine random_engine(rand());
                std::uniform_int_distribution<uint8_t> rand_in_neighbourhood(0, 255);

                // We need to account for the following corner cases:
                // - the point is on the left/right edge of the image -> select a point on the right/left semi-plane
                // - the point is on the top/bottom edge of the image -> select a point on the bottom/top semi-plane
                //
                // Corners do not require any further special casing.

                cv::Point neighbour = pt;
                if (pt.x == 0) {
                    neighbour.x += rand_in_neighbourhood(random_engine) % 2;
                } else if (neighbour.x == size.width) {
                    neighbour.x -= rand_in_neighbourhood(random_engine) % 2;
                } else {
                    neighbour.x += rand_in_neighbourhood(random_engine) % 3 - 1;
                }

                if (pt.y == 0) {
                    neighbour.y += rand_in_neighbourhood(random_engine) % 2;
                } else if (neighbour.y == size.height) {
                    neighbour.y -= rand_in_neighbourhood(random_engine) % 2;
                } else {
                    neighbour.y += rand_in_neighbourhood(random_engine) % 3 - 1;
                }

                return neighbour;
            }


            BackgroundExtraction::BackgroundExtraction() {
                parametersHelper.AddParameter("Background Extraction", "ForegroundLabel",
                                              parameters.foregroundLabel, DefaultParameters.foregroundLabel);
                parametersHelper.AddParameter("Background Extraction", "BackgroundLabel",
                                              parameters.backgroundLabel, DefaultParameters.backgroundLabel);
                parametersHelper.AddParameter("Background Extraction", "SamplesPerPixel",
                                              parameters.samplesPerPixel, DefaultParameters.samplesPerPixel);
                parametersHelper.AddParameter("Background Extraction", "DistanceThreshold",
                                              parameters.distanceThreshold, DefaultParameters.distanceThreshold);
                parametersHelper.AddParameter("Background Extraction", "SamplesPerPixel",
                                              parameters.requiredMatches, DefaultParameters.requiredMatches);
                parametersHelper.AddParameter("Background Extraction", "RequiredMatches",
                                              parameters.samplesPerPixel, DefaultParameters.samplesPerPixel);
                parametersHelper.AddParameter("Background Extraction", "SubsamplingFactor",
                                              parameters.subsamplingFactor, DefaultParameters.subsamplingFactor);
            }

            void BackgroundExtraction::configure() {
                parametersHelper.ReadFile(configurationFilePath);
                validateParameters();
            }

            void BackgroundExtraction::process() {
                Converters::FrameToMatConverter FrameToMat;
                Converters::MatToFrameConverter MatToFrame;

                cv::Mat inputImage = FrameToMat.Convert(&inImage);
                validateInputs();

                cv::Mat segmentation(inputImage.size(), CV_8UC1, parameters.foregroundLabel);
                if (_backgroundModel.empty()) {
                    initialise_background_model(inputImage, _backgroundModel);
                }

                classify_and_update(inputImage, _backgroundModel, segmentation);

                FrameConstPtr new_segmentation = MatToFrame.Convert(segmentation);
                Copy(*new_segmentation, outImage);
                delete new_segmentation;
            }


            void BackgroundExtraction::reset() {
                _backgroundModel.setTo(cv::Scalar(0, 0, 0));
            }


            void BackgroundExtraction::validateParameters() const {
                ASSERT(parameters.backgroundLabel != parameters.foregroundLabel,
                       "Foreground and background labels must be different");
            }


            void BackgroundExtraction::validateInputs() const {
                Validators::Frame::NotEmpty(inImage);
                Validators::Frame::HasFormatIn(inImage, {FrameWrapper::FrameMode::asn1Sccmode_GRAY});
                Validators::Frame::HasStatus(inImage, FrameWrapper::STATUS_VALID);
                Validators::Frame::HasDepthOf(inImage, Array3DWrapper::Array3DDepth::asn1Sccdepth_8U);
            }


            void BackgroundExtraction::initialise_background_model(
                    const cv::Mat &frame, cv::Mat &background_model) const {
                ASSERT(background_model.empty(),
                       "BackgroundSubtraction: Called BackgroundExtraction::initialise twice");
                background_model = cv::Mat({frame.rows, frame.cols, parameters.samplesPerPixel}, CV_8UC1);

                // The intialisation of the background model considers that small regions of the frame are similar and that the
                // value of each pixel of the region is a realisation of the same random variable. This allows us to simply sample
                // the neighbourhood of each pixel to build its background model instead of having to wait for many images.
                // To avoid overfitting to a small region we must have at least as many points in the regions as we have pixels
                // samples in the background model.
                const auto window_size = static_cast<uint8_t>(sqrt(parameters.samplesPerPixel) + 1);
                const auto half_window = window_size / static_cast<uint8_t>(2);

                ASSERT(frame.cols > window_size && frame.rows > window_size,
                       "BackgroundExtraction: Frame too small to properly initialize classifier");

                // pick points from the neighbourhood at random until we fill the model
                std::random_device rand;
                std::default_random_engine random_engine(rand());
                std::uniform_int_distribution<int32_t> rand_int(-half_window, +half_window);

                const cv::Size imsize = frame.size();

                for (int32_t row = 0; row < frame.rows; ++row) {
                    for (int32_t col = 0; col < frame.cols; ++col) {
                        for (uint32_t cnt = 0; cnt < parameters.samplesPerPixel; ++cnt) {
                            const auto neighbour = getPointFromNeighbourhoodOf({col, row}, imsize);
                            background_model.at<uint8_t>(row, col, cnt) = frame.at<uint8_t>(neighbour);
                        }
                    }
                }
            }


            void BackgroundExtraction::classify_and_update(
                    const cv::Mat &frame, cv::Mat &background_model, cv::Mat &segmentation) const {
                std::random_device rand;
                std::default_random_engine random_engine(rand());
                std::uniform_int_distribution<int32_t> rand_sample(1, parameters.samplesPerPixel);
                std::uniform_int_distribution<int32_t> rand_subsample(1, parameters.subsamplingFactor);

                const cv::Size imsize = frame.size();

                // In the canonical implementation the calssification and update steps are separate.
                // First you classify then you update. Here, to avoid iterating over the entire image
                // twice we do them both at the same time.
                // In practice this is not a problem, the temporal subsampling is performed after the
                // classification and the spatial samplig which may provide current information to
                // pixels that have not been classified yet is infrequent enough to not affect the
                // results of the classification.

                for (int32_t row = 0; row < frame.rows; ++row) {
                    for (int32_t col = 0; col < frame.cols; ++col) {
                        uint8_t matches = 0;
                        const auto frame_px = frame.at<uint8_t>(row, col);

                        for (uint8_t comparisons = 0; comparisons < parameters.samplesPerPixel; ++comparisons) {

                            // Compute the number of matches for each pixel
                            const auto sample = background_model.at<uint8_t>(row, col, comparisons);
                            const auto distance = std::abs(frame_px - sample);

                            if (distance < parameters.distanceThreshold) {
                                matches += 1;

                                if (matches >= parameters.requiredMatches) {
                                    segmentation.at<uint8_t>(row, col) = parameters.backgroundLabel;

                                    // Temporal subsampling
                                    if (rand_subsample(random_engine) == 1) {
                                        const int32_t sample_to_discard = rand_sample(random_engine);
                                        background_model.at<uint8_t>(row, col, sample_to_discard) = frame.at<uint8_t>(
                                                row,
                                                col);
                                    }

                                    // Spatial subsampling
                                    if (rand_subsample(random_engine) == 1) {
                                        const auto dst = getPointFromNeighbourhoodOf({col, row}, imsize);
                                        const int32_t sample_to_discard = rand_sample(random_engine);
                                        background_model.at<uint8_t>(dst.y, dst.x,
                                                                     sample_to_discard) = frame.at<uint8_t>(
                                                row, col);
                                    }

                                    break;  // break to for cols ...
                                }
                            }
                        } // end for comparisons
                    } // end for cols
                } // end for rows


                // Post Processing Step - Denoising
                // Could be replaced or enhanced with an open/close cycle
                cv::Mat mask;
                cv::GaussianBlur(segmentation, mask, cv::Size(9, 9), 7, 7);
                cv::threshold(mask, mask, /* thresh = */ 30, /* maxval = */ 255, cv::THRESH_BINARY);
                segmentation = segmentation & mask;
            }
        } // End namespace ImageFiltering
    } // End namespace DFN
} // End namespace CDFF
