#ifndef NORMALMAP2D_INTERFACE_HPP
#define NORMALMAP2D_INTERFACE_HPP
#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
// added 
#include <FrameToMatConverter.hpp>

namespace dfn_ci {
    class NormalMap2DInterface : public DFNCommonInterface
    {
        public:
            NormalMap2DInterface();
            virtual ~NormalMap2DInterface();
            /**
            * Send value to input port frame
            * @param frame, a binary 2D image representing edges.
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port NormalMap
            * @return NormalMap, This is the matrix representing the hough space of lines extracted from the image
            */
            virtual FrameWrapper::FrameConstPtr NormalMapOutput();

        protected:
            FrameWrapper::FrameConstPtr inImage;
            FrameWrapper::FrameConstPtr outNormalMap;
    };
}
#endif