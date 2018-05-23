#ifndef KMEANS2D_INTERFACE_HPP
#define KMEANS2D_INTERFACE_HPP
#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
// added 
#include <FrameToMatConverter.hpp>

namespace dfn_ci {
    class KMeans2DInterface : public DFNCommonInterface
    {
        public:
            KMeans2DInterface();
            virtual ~KMeans2DInterface();
            /**
            * Send value to input port frame
            * @param frame, a binary 2D image representing edges.
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port KMeansImage
            * @return KMeansImage, This is the matrix representing the hough space of lines extracted from the image
            */
            virtual FrameWrapper::FrameConstPtr KMeansImageOutput();

        protected:
            FrameWrapper::FrameConstPtr inImage;
            FrameWrapper::FrameConstPtr outKMeansImage;
    };
}
#endif