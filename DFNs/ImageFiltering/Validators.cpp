#include "Validators.hpp"

namespace CDFF {
    namespace DFN{
        namespace ImageFiltering {


            namespace Validators {
                namespace Frame {
                    void NotEmpty(const FrameWrapper::Frame &frame) {
                        ASSERT(frame.data.cols > 0 && frame.data.rows > 0,
                               "Validators/Frame - Image should not be empty");
                    }

                    void HasFormatIn(const FrameWrapper::Frame &frame,
                                     const std::vector<FrameWrapper::FrameMode> &frameModes) {
                        const size_t mode_count = frameModes.size();
                        for (size_t idx = 0; idx < mode_count; ++idx) {
                            if (frame.metadata.mode == frameModes[idx]) {
                                return;
                            }
                        }

                        ASSERT(false, "Frame Mode not in approved list");
                    }

                    void HasStatus(const FrameWrapper::Frame &frame, FrameWrapper::FrameStatus status) {
                        ASSERT(frame.metadata.status == status, "Validators/Status - Invalid frame status");
                    }

                    void HasDepthOf(const FrameWrapper::Frame &frame, Array3DWrapper::Array3DDepth depth) {
                        ASSERT(frame.data.depth == depth, "Validators/Frame - Image has wrong depth");
                    }
                }
            }
        }
    }
}