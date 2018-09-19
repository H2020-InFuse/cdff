/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BACKGROUNDEXTRACTION_HPP
#define BACKGROUNDEXTRACTION_HPP

#include "ImageFilteringInterface.hpp"

#include <opencv2/core.hpp>

#include <Helpers/ParametersListHelper.hpp>

namespace CDFF {
    namespace DFN {


        cv::Point2i getPointFromNeighbourhoodOf(const cv::Point2i &pt, const cv::Size &size);

    /**
     * Segment and image to identify the foreground and the background.
     *
     * The class implements ViBe
     */
    class BackgroundExtraction : public ImageFilteringInterface
    {
        struct Parameters {
            /// The value of foreground points in the segmentation mask
            uint8_t foregroundLabel = 255;

                /// The value of background points in the segmentation mask
                uint8_t backgroundLabel = 0;

                /// How many past samples should be stored in the background model
                /// for each pixel
                uint8_t samplesPerPixel = 20;

                /// What is the maximum distance between pixel values that should be
                /// considered similar ?
                uint8_t distanceThreshold = 20;

                /// How many samples in the background model should be matched by
                /// the current pixel for us to consider that it is a a background
                /// pixel.
                uint8_t requiredMatches = 2;

                /// At what frequency should samples in the background model be
                /// updated ?
                /// This covers both spatial and temporal diffusion where the
                /// probability that the current pixel value will be inserted into
                /// the background model is '1 / subsamplingFactor'.
                uint8_t subsamplingFactor = 16;
            };

        public:

        static const Parameters DefaultParameters;

        BackgroundExtraction();

            void configure() override;

            void process() override;

            void reset();

        private:


            void validateParameters() const;

            void validateInputs() const;

            // TODO: Immediate / Fast initialisation
            void initialise_background_model(const cv::Mat &frame, cv::Mat &background_model) const;

            // TODO <tch> expose other distance functions
            void classify_and_update(const cv::Mat &image, cv::Mat &background_model, cv::Mat &segmentation) const;


            // State Variables

            cv::Mat _backgroundModel;

            // Plumbing

        Helpers::ParametersListHelper parametersHelper;
        Parameters parameters;
        };
    }
}

#endif // BACKGROUNDEXTRACTION_HPP

/** @} */
