/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEFILTERING_BACKGROUNDEXTRACTION_HPP
#define IMAGEFILTERING_BACKGROUNDEXTRACTION_HPP

#include "ImageFilteringInterface.hpp"

#include <opencv2/core.hpp>

#include <Helpers/ParametersListHelper.hpp>

namespace CDFF {
    namespace DFN {
        namespace ImageFiltering {
            /**
             * Select a point in the 8 connected neighbourdhood of `pt` in an image of `size` pixels
             * big. The method is edge aware and will not pick points outside of the frame.
             *
             * @param pt The point around which to pick a neighbour
             * @param size
             *      The size of the image (used to ensure that the selected point is in the image)
             * @return A point in the 8-connected neighbourhood of `pt`
             */
            cv::Point2i getPointFromNeighbourhoodOf(const cv::Point2i &pt, const cv::Size &size);

            /**
             * Segment and image to identify the foreground and the background.
             *
             * The class implements background extraction using the ViBe algorithm [1]. The algorithm can
             * be divided into 3 phases:
             *
             * 1 - Initialisation
             *
             * A statistical model of each pixel is estimated by sampling its neighbourhood and storing
             * the values in a buffer, the _background buffer_. The size of this buffer is controlled by
             * the `samplesPerPixel` parameter. The larger the buffer the more precise the model becomes
             * but the higher the storage and computational demands
             *
             * 2 - Classification
             *
             * The current value of each pixel is compared to the values in its background buffer. If the
             * distance between it and at least `requiredMatches` values in the buffer is less than
             * `distanceThreshold` then the pixel is considered as being part of the background and
             * assigned `backgroundLabel`, otherwise it is part of the foreground and assigned
             * `foregroundLabel`.
             *
             * 3 - Model Update
             *
             * To ensure that the pixel model stays current it is updated with spatial and temporal sampling.
             * Only pixels that are part of the background are updated. Pixels in the foreground will
             * progressively received information from the spatial subsampling process.
             *
             * The temporal sampling is a random process in which there is a `1 / subsamplingFactor` chance
             * that the current pixel value will replace one of the values in its background buffer.
             *
             * The spatial sampling diffuses pixels into their 8-connected neighbourhood. There is a
             * `1 / subsamplingFactor` chance that the current pixel will be added to the background
             * buffer of a randomly selected neighbour.
             *
             * The default parameters are tuned for for video feeds with approximately 15 frames/second.
             * This may cause issues with lower frequency cameras and may require adjustment. Specifically
             * the subsampling factor should be lowered to ensure that new information propagates more
             * rapidly.
             *
             * If the computational requirements [rove to be too onerous lowering the `samplesPerPixel`
             * will not affect the quality of the output too much.
             *
             * [1]: Barnich, O., & Van Droogenbroeck, M. (2011). ViBe: A universal background subtraction
             *      algorithm for video sequences. IEEE Transactions on Image processing, 20(6), 1709-1724.
             */
            class BackgroundExtraction : public ImageFilteringInterface {
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

                /**
                 * Initialiase the background model for each pixel in the source image. See the class
                 * description for more information.
                 *
                 * @param frame The frame to use a the starting point
                 * @param background_model A `samplesPerPixel` deep image containing all the background model.
                 */
                void initialise_background_model(const cv::Mat &frame, cv::Mat &background_model) const;

                /**
                 * Classify each pixel into two bins: foreground or background and update the background model.
                 * For more information see the class description.
                 *
                 * @param image The image to classify
                 * @param background_model The background model
                 * @param segmentation The resulting segmentation map with two classes: foreground and background
                 */
                void classify_and_update(const cv::Mat &image, cv::Mat &background_model, cv::Mat &segmentation) const;


                // State Variables

                /// This is the matrix containing the background model for each pixel.
                cv::Mat _backgroundModel;

                // Plumbing

                Helpers::ParametersListHelper parametersHelper;
                Parameters parameters;
            };
        } // End namespace ImageFiltering
    } // End namespace DFN
} // End namespace CDFF


#endif // IMAGEFILTERING_BACKGROUNDEXTRACTION_HPP

/** @} */
