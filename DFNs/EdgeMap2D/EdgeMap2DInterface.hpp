#ifndef EDGEMAP2D_INTERFACE_HPP
#define EDGEMAP2D_INTERFACE_HPP
#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
// added 
#include <FrameToMatConverter.hpp>

namespace dfn_ci {
    class EdgeMap2DInterface : public DFNCommonInterface
    {
        public:
            EdgeMap2DInterface();
            virtual ~EdgeMap2DInterface();
            /**
            * Send value to input port frame
            * @param frame, a binary 2D image representing edges.
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port EdgeMap
            * @return EdgeMap, This is the matrix representing the hough space of lines extracted from the image
            */
            virtual FrameWrapper::FrameConstPtr EdgeMapOutput();

        protected:
            FrameWrapper::FrameConstPtr inImage;
            FrameWrapper::FrameConstPtr outEdgeMap;
    };
}
#endif