/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEFILTERING_IMAGEFILTERINGINTERFACE_HPP
#define IMAGEFILTERING_IMAGEFILTERINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that applies an image processing filter to a 2D image
     */
    class ImageFilteringInterface : public DFNCommonInterface
    {
        public:

            ImageFilteringInterface();
            virtual ~ImageFilteringInterface();

            /**
             * Send value to input port "image"
             * @param image
             *     2D image captured by a camera
             */
            virtual void imageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "image"
             * @return image
             *     same image, filtered
             */
            virtual const asn1SccFrame& imageOutput() const;

        protected:

            asn1SccFrame inImage;
            asn1SccFrame outImage;
    };
}
}

#endif // IMAGEFILTERING_IMAGEFILTERINGINTERFACE_HPP

/** @} */
