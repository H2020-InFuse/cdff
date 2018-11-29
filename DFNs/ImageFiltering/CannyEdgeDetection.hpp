//
// Created by tch on 06.07.18.
//

#ifndef CDFF_CANNYEDGEDETECTION_H
#define CDFF_CANNYEDGEDETECTION_H

#include "ImageFilteringInterface.hpp"

#include <Types/CPP/Frame.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
    namespace DFN
    {
        namespace ImageFiltering
        {

            /**
         * This DFN will detect edges in an image using the Canny edge detector.
         *
         * The method can be broken up into a series of discrete steps:
         *
         * 1. De-noise the image using a gaussian blur filter
         * 2. Locate edges based on the horizontal and vertical gradients
         * 3. Thin the detected edges to a width of 1 pixel
         * 4. Classify pixels into weak and strong edges. Strong edge pixels have a
         *    gradient higher than the _Canny Upper Threshold_ and weak edges have a
         *    gradient that is higher than the _Canny Lower Threshold_ but less than
         *    the _Canny Upper Threshold_. Other pixels are discarded.
         * 5. Discard edge pixels due to noise using blob analysis on the weak edge
         *    pixels. Real edge pixels tend to be connected whilst those due to
         *    noise aren't.
         *
         * The critical part of the method is the choice of the low and high
         * thresholds. They are content-dependent and best tuned manually for each
         * different scene.
         */
            class CannyEdgeDetection : public ImageFilteringInterface {

                /// Set of parameters for the CannyEdgeDetection DFN
                struct Parameters {
                    /// The size of the gaussian blur kernel used to denoise the input.
                    /// Should be an odd number. Usually between 3 and 7.
                    int NoiseReductionKernelSize = 5;

                    /// The low threshold used to classify the edge pixels into weak and strong sets.
                    double CannyLowThreshold = 80.;

                    ///  The high threshold used to classify the edge pixels into weak and strong sets.
                    double CannyHighThreshold = 100.;
                };

            public:
                static const Parameters DefaultParameters;

                CannyEdgeDetection();

                void configure() override;

                void process() override;

            private:

                void ValidateParameters();

                void ValidateInputs(const FrameWrapper::Frame &frame);

                Parameters parameters;
                Helpers::ParametersListHelper parametersHelper;
            };
        }
    }
}
#endif //CDFF_CANNYEDGEDETECTION_H
