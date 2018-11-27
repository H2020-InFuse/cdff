//
// Created by tch on 06.07.18.
//

#include "EdgeDetection.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Types/C/Frame.h>
#include <Types/CPP/Frame.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include "Validators/Frame.hpp"
#include "Validators/Number.hpp"

namespace CDFF
{
    namespace DFN
    {
        namespace ImageFiltering {

            const EdgeDetection::Parameters EdgeDetection::DefaultParameters = {};

            EdgeDetection::EdgeDetection() {
                parametersHelper.AddParameter(
                        "EdgeDetection", "NoiseReductionKernelSize",
                        parameters.NoiseReductionKernelSize, DefaultParameters.NoiseReductionKernelSize);
                parametersHelper.AddParameter(
                        "EdgeDetection", "CannyLowThreshold",
                        parameters.CannyLowThreshold, DefaultParameters.CannyLowThreshold);
                parametersHelper.AddParameter(
                        "EdgeDetection", "CannyHighThreshold",
                        parameters.CannyHighThreshold, DefaultParameters.CannyHighThreshold);
            }

            void EdgeDetection::configure() {
                parametersHelper.ReadFile(configurationFilePath);
                ValidateParameters();
            }

            void EdgeDetection::process() {
                ValidateInputs(inImage);
                cv::Mat inputImage = Converters::FrameToMatConverter().Convert(&inImage);

                if (inImage.metadata.mode == FrameWrapper::FrameMode::asn1Sccmode_RGB) {
                    cv::cvtColor(inputImage, inputImage, cv::COLOR_RGB2GRAY);
                }

                int denoise_range = parameters.NoiseReductionKernelSize;
                cv::blur(inputImage, inputImage, cv::Size(denoise_range, denoise_range));
                cv::Canny(inputImage, inputImage, parameters.CannyLowThreshold, parameters.CannyHighThreshold);

                FrameWrapper::FrameConstPtr outputImage =
                        Converters::MatToFrameConverter().Convert(inputImage);
                FrameWrapper::Copy(*outputImage, outImage);
                delete outputImage;
            }


            void EdgeDetection::ValidateParameters() {
                Validators::Number::IsOdd(parameters.NoiseReductionKernelSize);
                Validators::Number::IsGreaterThan(parameters.NoiseReductionKernelSize, 1);
                Validators::Number::IsGreaterThan(parameters.CannyHighThreshold, parameters.CannyLowThreshold);
            }

            void EdgeDetection::ValidateInputs(FrameWrapper::Frame const &frame) {
                Validators::Frame::NotEmpty(frame);
                Validators::Frame::HasFormatIn(frame, {
                        FrameWrapper::FrameMode::asn1Sccmode_GRAY,
                        FrameWrapper::FrameMode::asn1Sccmode_RGB
                });
            }
        }
    }
}