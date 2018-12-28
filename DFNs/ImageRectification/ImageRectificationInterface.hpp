/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGERECTIFICATION_IMAGERECTIFICATIONINTERFACE_HPP
#define IMAGERECTIFICATION_IMAGERECTIFICATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that rectifies an image
     */
    class ImageRectificationInterface : public DFNCommonInterface
    {
        public:

            ImageRectificationInterface();
            virtual ~ImageRectificationInterface();

            /**
             * Send value to input port "originalImage"
             * @param originalImage
             *     Original distorted image
             */
            virtual void originalImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "rectifiedImage"
             * @return rectifiedImage
             *     Undistorted/rectified image
             */
            virtual const asn1SccFrame& rectifiedImageOutput() const;

        protected:

            asn1SccFrame inOriginalImage;
            asn1SccFrame outRectifiedImage;
    };
}
}

#endif // IMAGERECTIFICATION_IMAGERECTIFICATIONINTERFACE_HPP

/** @} */
