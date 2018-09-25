//
// Created by tch on 25/09/18.
//

#ifndef CDFF_VAIDATORS_IMAGE_HPP
#define CDFF_VAIDATORS_IMAGE_HPP

#include <vector>

#include "Types/CPP/Frame.hpp"

namespace CDFF {
    namespace Validators {
        namespace Frame {
            void NotEmpty(const FrameWrapper::Frame &frame);

            void HasFormatIn(const FrameWrapper::Frame &frame,
                             const std::vector <FrameWrapper::FrameMode> &frameModes);

            void HasStatus(const FrameWrapper::Frame &frame, FrameWrapper::FrameStatus status);

            void HasDepthOf(const FrameWrapper::Frame &frame, Array3DWrapper::Array3DDepth depth);
        }

    }
}

#endif //CDFF_VAIDATORS_IMAGE_HPP
