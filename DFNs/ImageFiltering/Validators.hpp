#pragma once

#include <Frame.hpp>
using namespace FrameWrapper;

namespace CDFF {
    namespace DFN{
        namespace ImageFiltering {
            namespace Validators {
                namespace Frame {
                    void NotEmpty(const FrameWrapper::Frame &frame);

                    void HasFormatIn(const FrameWrapper::Frame &frame,
                                     const std::vector<FrameWrapper::FrameMode> &frameModes);

                    void HasStatus(const FrameWrapper::Frame &frame, FrameWrapper::FrameStatus status);

                    void HasDepthOf(const FrameWrapper::Frame &frame, Array3DWrapper::Array3DDepth depth);
                }

                namespace Number {
                    template<class NumberType>
                    void GreaterThan(NumberType value, NumberType min_val) {
                        ASSERT(value > min_val, "Validators/Number - Should be greater than specified value");
                    }

                    template<typename NumberType>
                    void IsOdd(NumberType value) {
                        ASSERT(value % 2 != 0, "Validators/Number - Should be odd");
                    }
                }
            }
        }
    }
}
