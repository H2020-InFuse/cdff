//
// Created by tch on 06.07.18.
//

#include "EdgeDetection.hpp"

#include <opencv2/core.hpp>

#include <Types/C/Frame.h>
#include <Types/CPP/Frame.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


namespace Validators {
    namespace Number {
        template <class NumberType>
        void GreaterThan(NumberType value, NumberType min_val) {
            ASSERT(value > min_val, "Validators/Number - Should be greater than specified value");
        }

        template<typename NumberType>
        void IsOdd(NumberType value) {
            ASSERT(value % 2 != 0, "Validators/Number - Should be odd");
        }
    }

    namespace Frame {
        void NotEmpty(const FrameWrapper::Frame& frame) {
            const FrameWrapper::FrameSize& frame_size = frame.datasize;
            ASSERT(frame_size.width > 0 && frame_size.height > 0,
                "Validators/Frame - Image should not be empty");
        }

        void FormatIn(const FrameWrapper::Frame& frame, std::vector<asn1SccFrame_mode_t> frameModes) {
            const size_t mode_count = frameModes.size();
            for (size_t idx = 0; idx < mode_count; ++idx) {
                if (frame.frame_mode == frameModes[idx])
                {
                    return;
                }
            }

            ASSERT(false, "Frame Mode not in approved list");
        }
    }
}

namespace dfn_ci
{
    const EdgeDetection::Parameters EdgeDetection::DefaultParameters = {};

    EdgeDetection::EdgeDetection()
    {
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

    void EdgeDetection::configure()
    {
        parametersHelper.ReadFile(configurationFilePath);
        ValidateParameters();

        std::cout << "New Edge Detection Parameters: \n"
                  << "  - Noise Reduction Kernel Size: " << static_cast<int>(parameters.NoiseReductionKernelSize) << "\n"
                  << "  - Canny Low Threshold: " << parameters.CannyLowThreshold << "\n"
                  << "  - Canny High Threshold: " << parameters.CannyHighThreshold << "\n";
    }

    void EdgeDetection::process()
    {
        ValidateInputs(inImage);
        cv::Mat inputImage = Converters::FrameToMatConverter().Convert(&inImage);

        if (inImage.frame_mode == asn1Sccmode_rgb)
        {
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


    void EdgeDetection::ValidateParameters()
    {
        Validators::Number::IsOdd(parameters.NoiseReductionKernelSize);
        Validators::Number::GreaterThan(parameters.NoiseReductionKernelSize, 1);
        Validators::Number::GreaterThan(parameters.CannyHighThreshold, parameters.CannyLowThreshold);
    }

    void EdgeDetection::ValidateInputs(FrameWrapper::Frame const &frame)
    {
        Validators::Frame::NotEmpty(frame);
        Validators::Frame::FormatIn(frame, {
            asn1Sccmode_grayscale, asn1Sccmode_rgb });
    }


}